#!/usr/bin/env python3
"""
FSDS Stanley Controller — SLOW MODE (FINAL)

✔ FSDS-native ControlCommand
✔ Continuous publishing (required by FSDS)
✔ No brake latch
✔ Soft-start + soft-slowdown
✔ Target crawl speed ≈ 0.02
✔ Stable Stanley control at very low speed
"""

import math
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand


class FSDSStanleyController(Node):
    def __init__(self):
        super().__init__('fsds_stanley_controller')

        # ================= CONTROL PARAMETERS =================
        self.k = 0.8                     # Stanley gain (reduced for slow speed)
        self.max_steer = 1.0             # FSDS normalized [-1, 1]

        # --- SPEED CONTROL ---
        self.start_throttle = 0.12       # Minimum FSDS movement threshold
        self.target_throttle = 0.02      # YOUR requested crawl speed
        self.throttle_decay = 0.002      # Smooth ramp-down rate
        self.throttle = self.start_throttle

        # ================= STATE =================
        self.pose = None
        self.yaw = 0.0
        self.centerline = []

        # ================= SUBSCRIPTIONS =================
        self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_cb,
            10
        )

        self.create_subscription(
            MarkerArray,
            '/local_centerline',
            self.centerline_cb,
            10
        )

        # ================= PUBLISHER =================
        self.cmd_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10
        )

        # Publish continuously at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("✅ FSDS Stanley Controller running (SLOW MODE)")

    # ======================================================
    # CALLBACKS
    # ======================================================

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        q = self.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def centerline_cb(self, msg):
        pts = []
        for m in msg.markers:
            if m.action != m.ADD:
                continue
            pts.append((m.pose.position.x, m.pose.position.y))
        self.centerline = sorted(pts, key=lambda p: p[0])

    # ======================================================
    # CONTROL LOOP
    # ======================================================

    def control_loop(self):
        # Always publish (FSDS requirement)
        cmd = ControlCommand()
        cmd.brake = 0.0

        # ---------- SOFT START / SLOWDOWN ----------
        if self.throttle > self.target_throttle:
            self.throttle -= self.throttle_decay
            self.throttle = max(self.throttle, self.target_throttle)

        # ---------- NO STATE YET ----------
        if self.pose is None or len(self.centerline) < 2:
            cmd.throttle = self.throttle
            cmd.steering = 0.0
            self.cmd_pub.publish(cmd)
            return

        # ---------- LOOKAHEAD ----------
        target = None
        for x, y in self.centerline:
            if x > 2.0:
                target = (x, y)
                break

        if target is None:
            cmd.throttle = self.throttle
            cmd.steering = 0.0
            self.cmd_pub.publish(cmd)
            return

        # ---------- STANLEY CONTROL ----------
        dx = target[0] - self.pose.position.x
        dy = target[1] - self.pose.position.y

        heading_error = math.atan2(dy, dx) - self.yaw
        heading_error = self.normalize_angle(heading_error)

        cte = dy * math.cos(self.yaw) - dx * math.sin(self.yaw)

        # Protect against near-zero division
        speed_term = max(self.throttle, 0.05)
        steer = heading_error + math.atan2(self.k * cte, speed_term)

        steer = max(-self.max_steer, min(self.max_steer, steer))

        # ---------- PUBLISH ----------
        cmd.throttle = self.throttle
        cmd.steering = steer
        self.cmd_pub.publish(cmd)

    # ======================================================
    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = FSDSStanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
FSDS Stanley Controller — FINAL (FSDS-BUILD COMPATIBLE)

✔ Uses fs_msgs/ControlCommand
✔ NO SetControlMode (not supported in your FSDS)
✔ NO brake latch
✔ Continuous throttle from startup
✔ Identical control contract to keyboard_control.py
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

        # ===== PARAMETERS =====
        self.k = 1.0
        self.throttle = 0.35          # MUST be > 0.3 for FSDS
        self.max_steer = 1.0

        # ===== STATE =====
        self.pose = None
        self.yaw = 0.0
        self.centerline = []

        # ===== SUBSCRIPTIONS =====
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

        # ===== PUBLISHER =====
        self.cmd_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10
        )

        # Publish continuously at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("✅ FSDS Stanley Controller RUNNING")

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
    def control_loop(self):
        # ALWAYS publish — like keyboard_control.py
        cmd = ControlCommand()
        cmd.brake = 0.0

        if self.pose is None or len(self.centerline) < 2:
            cmd.throttle = self.throttle
            cmd.steering = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Lookahead point ~2m
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

        dx = target[0] - self.pose.position.x
        dy = target[1] - self.pose.position.y

        heading_error = math.atan2(dy, dx) - self.yaw
        heading_error = self.normalize(heading_error)

        cte = dy * math.cos(self.yaw) - dx * math.sin(self.yaw)

        steer = heading_error + math.atan2(self.k * cte, self.throttle)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        cmd.throttle = self.throttle
        cmd.steering = steer

        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = FSDSStanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


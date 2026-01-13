#!/usr/bin/env python3
"""
FSDS Stanley Controller — GEOMETRY CORRECT (FINAL)

✔ Closest-segment projection
✔ Proper signed CTE
✔ Path tangent heading error
✔ Slow/snail mode (~0.02)
✔ FSDS-safe control publishing
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

        # ================= PARAMETERS =================
        self.k = 1.0
        self.max_steer = 1.0

        # Slow mode
        self.start_throttle = 0.12
        self.target_throttle = 0.02
        self.throttle_decay = 0.002
        self.throttle = self.start_throttle

        # ================= STATE =================
        self.pose = None
        self.yaw = 0.0
        self.centerline = []

        # ================= SUBSCRIBERS =================
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

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("✅ Stanley Controller (geometry-correct) running")

    # =====================================================
    # CALLBACKS
    # =====================================================

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
            if m.action == m.ADD:
                pts.append((m.pose.position.x, m.pose.position.y))
        self.centerline = pts

    # =====================================================
    # GEOMETRY HELPERS
    # =====================================================

    def closest_segment(self, path, px, py):
        min_dist = float('inf')
        best_i = 0

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            vx = x2 - x1
            vy = y2 - y1
            wx = px - x1
            wy = py - y1

            seg_len2 = vx * vx + vy * vy
            if seg_len2 == 0:
                continue

            t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len2))
            proj_x = x1 + t * vx
            proj_y = y1 + t * vy

            dx = px - proj_x
            dy = py - proj_y
            dist = dx * dx + dy * dy

            if dist < min_dist:
                min_dist = dist
                best_i = i

        return best_i

    @staticmethod
    def normalize(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # =====================================================
    # CONTROL LOOP
    # =====================================================

    def control_loop(self):
        cmd = ControlCommand()
        cmd.brake = 0.0

        # Smooth throttle ramp
        if self.throttle > self.target_throttle:
            self.throttle = max(
                self.target_throttle,
                self.throttle - self.throttle_decay
            )

        if self.pose is None or len(self.centerline) < 3:
            cmd.throttle = self.throttle
            cmd.steering = 0.0
            self.cmd_pub.publish(cmd)
            return

        px = self.pose.position.x
        py = self.pose.position.y

        # --------- Closest segment projection ---------
        i = self.closest_segment(self.centerline, px, py)
        x1, y1 = self.centerline[i]
        x2, y2 = self.centerline[i + 1]

        # Path tangent
        path_yaw = math.atan2(y2 - y1, x2 - x1)

        # Signed cross-track error
        dx = px - x1
        dy = py - y1
        cte = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        # Heading error
        heading_error = self.normalize(path_yaw - self.yaw)

        # Stanley control law
        speed_term = max(self.throttle, 0.05)
        steer = heading_error + math.atan2(self.k * cte, speed_term)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        cmd.throttle = self.throttle
        cmd.steering = steer
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FSDSStanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


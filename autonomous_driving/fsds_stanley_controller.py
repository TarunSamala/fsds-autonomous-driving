#!/usr/bin/env python3
"""
FSDS Stanley Controller — Phase 4

Consumes local centerline markers and outputs steering + throttle.
Ego-frame, low-speed, stability-first controller.
"""

import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped


class StanleyController(Node):
    def __init__(self):
        super().__init__('fsds_stanley_controller')

        # ---------- Parameters ----------
        self.k = 1.2                 # Stanley gain
        self.max_steer = 0.4         # radians
        self.target_speed = 2.0      # m/s equivalent throttle
        self.wheelbase = 0.32        # FSDS approx

        # ---------- State ----------
        self.centerline = []

        # ---------- Subscribers ----------
        self.create_subscription(
            MarkerArray,
            '/local_centerline',
            self.centerline_cb,
            10
        )

        # ---------- Publisher ----------
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            '/cmd_drive',
            10
        )

        self.timer = self.create_timer(0.05, self.control_step)

        self.get_logger().info("✅ Stanley Controller running")

    # ==========================================================
    # Callbacks
    # ==========================================================

    def centerline_cb(self, msg):
        points = []
        for m in msg.markers:
            if m.action != m.ADD:
                continue
            points.append((m.pose.position.x, m.pose.position.y))

        # Sort forward
        self.centerline = sorted(points, key=lambda p: p[0])

    # ==========================================================
    # Control logic
    # ==========================================================

    def control_step(self):
        if len(self.centerline) < 3:
            return

        # Pick a lookahead point (2–4 m ahead)
        lookahead = None
        for x, y in self.centerline:
            if x > 2.0:
                lookahead = (x, y)
                break

        if lookahead is None:
            return

        lx, ly = lookahead

        # Heading error (vehicle faces +X)
        heading_error = math.atan2(ly, lx)

        # Cross-track error
        cte = ly

        # Stanley term
        stanley_term = math.atan2(self.k * cte, self.target_speed)

        steer = heading_error + stanley_term

        # Clamp steering
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # Publish command
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.steering_angle = steer
        cmd.drive.speed = self.target_speed

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


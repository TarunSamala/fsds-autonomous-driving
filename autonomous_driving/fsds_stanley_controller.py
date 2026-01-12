#!/usr/bin/env python3
"""
FSDS Stanley Controller — Phase 4 (STABLE)

Low-speed, FSDS-calibrated Stanley controller.
"""

import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from fs_msgs.msg import ControlCommand


class StanleyController(Node):
    def __init__(self):
        super().__init__('fsds_stanley_controller')

        # ================= FSDS-CALIBRATED PARAMS =================
        self.k = 0.6                 # Stanley gain (LOW)
        self.throttle = 0.02         # MATCH keyboard driving
        self.max_steer = 0.35        # FSDS practical limit
        self.v_min = 0.05            # velocity floor for Stanley

        # ================= State =================
        self.centerline = []

        # ================= ROS =================
        self.create_subscription(
            MarkerArray,
            '/local_centerline',
            self.centerline_cb,
            10
        )

        self.cmd_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10
        )

        self.timer = self.create_timer(0.05, self.control_step)

        self.get_logger().info("✅ Stanley Controller running (FSDS stable)")

    # ------------------------------------------------------------

    def centerline_cb(self, msg):
        pts = []
        for m in msg.markers:
            if m.action != m.ADD:
                continue
            pts.append((m.pose.position.x, m.pose.position.y))

        self.centerline = sorted(pts, key=lambda p: p[0])

    # ------------------------------------------------------------

    def control_step(self):
        if len(self.centerline) < 3:
            return

        # Lookahead ~1.5–2.0 m
        target = None
        for x, y in self.centerline:
            if x > 1.5:
                target = (x, y)
                break

        if target is None:
            return

        lx, ly = target

        # Heading error
        heading_error = math.atan2(ly, lx)

        # Cross-track error (ego frame)
        cte = ly

        # Stanley term (stabilized)
        v = max(self.throttle, self.v_min)
        stanley_term = math.atan2(self.k * cte, v)

        steer = heading_error + stanley_term
        steer = max(-self.max_steer, min(self.max_steer, steer))

        cmd = ControlCommand()
        cmd.throttle = float(self.throttle)
        cmd.steering = float(steer)
        cmd.brake = 0.0

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


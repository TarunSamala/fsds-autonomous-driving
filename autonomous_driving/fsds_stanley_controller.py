#!/usr/bin/env python3
"""
FSDS Stanley Controller — FINAL PATCH

✔ Uses fs_msgs/ControlCommand (FSDS native)
✔ Arms FSDS API control explicitly
✔ Clears brake latch
✔ Kickstarts vehicle
✔ Continuous command publishing
✔ Compatible with your keyboard_control.py behavior
"""

import math
import time
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry

from fs_msgs.msg import ControlCommand
from fs_msgs.srv import SetControlMode


class FSDSStanleyController(Node):
    def __init__(self):
        super().__init__('fsds_stanley_controller')

        # ================= PARAMETERS =================
        self.k = 1.0                     # Stanley gain
        self.base_throttle = 0.30        # MUST be > 0.25 for FSDS
        self.max_steer = 1.0             # FSDS normalized [-1, 1]

        self.kickstart_throttle = 0.45
        self.kickstart_duration = 0.6    # seconds

        # ================= STATE =================
        self.centerline = []
        self.pose = None
        self.yaw = 0.0

        self.kickstart_done = False
        self.start_time = time.time()

        # ================= SUBSCRIBERS =================
        self.create_subscription(
            MarkerArray,
            '/local_centerline',
            self.centerline_cb,
            10
        )

        self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_cb,
            10
        )

        # ================= PUBLISHER =================
        self.cmd_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10
        )

        # ================= FSDS CONTROL MODE =================
        self.control_mode_client = self.create_client(
            SetControlMode,
            '/set_control_mode'
        )

        while not self.control_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /set_control_mode service...")

        self.arm_api_control()

        # ================= TIMER =================
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("✅ FSDS Stanley Controller ACTIVE")

    # ==========================================================
    # FSDS CONTROL ARMING
    # ==========================================================

    def arm_api_control(self):
        req = SetControlMode.Request()
        req.control_mode = SetControlMode.Request.API

        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("✅ FSDS API control armed")
        else:
            self.get_logger().error("❌ Failed to arm FSDS API control")

    # ==========================================================
    # CALLBACKS
    # ==========================================================

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

    # ==========================================================
    # CONTROL LOOP
    # ==========================================================

    def control_loop(self):
        if self.pose is None or len(self.centerline) < 2:
            self.publish_cmd(0.0, 0.0, 1.0)  # HOLD BRAKE
            return

        # ---------- KICKSTART ----------
        elapsed = time.time() - self.start_time
        if not self.kickstart_done and elapsed < self.kickstart_duration:
            self.publish_cmd(self.kickstart_throttle, 0.0, 0.0)
            return
        else:
            self.kickstart_done = True

        # ---------- STANLEY ----------
        x = self.pose.position.x
        y = self.pose.position.y

        # Choose nearest forward point
        target = None
        for px, py in self.centerline:
            if px > 1.5:
                target = (px, py)
                break

        if target is None:
            self.publish_cmd(0.0, 0.0, 0.5)
            return

        dx = target[0] - x
        dy = target[1] - y

        heading_error = math.atan2(dy, dx) - self.yaw
        heading_error = self.normalize_angle(heading_error)

        cte = dy * math.cos(self.yaw) - dx * math.sin(self.yaw)

        steer = heading_error + math.atan2(self.k * cte, self.base_throttle)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        self.publish_cmd(self.base_throttle, steer, 0.0)

    # ==========================================================
    # COMMAND PUBLISH
    # ==========================================================

    def publish_cmd(self, throttle, steering, brake):
        cmd = ControlCommand()
        cmd.throttle = float(throttle)
        cmd.steering = float(steering)
        cmd.brake = float(brake)
        self.cmd_pub.publish(cmd)

    # ==========================================================
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


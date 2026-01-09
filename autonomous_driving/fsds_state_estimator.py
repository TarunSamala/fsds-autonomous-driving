#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class FsdsStateEstimator(Node):
    def __init__(self):
        super().__init__('fsds_state_estimator')

        self.sub = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_cb,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/ego_state_debug',
            10
        )

        self.last_yaw = None
        self.last_time = None

        self.get_logger().info("✅ FSDS State Estimator started")

    def odom_cb(self, msg: Odometry):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation → yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # World-frame velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        # Ego-frame forward speed
        v = math.cos(yaw) * vx + math.sin(yaw) * vy

        # Yaw rate (finite difference)
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw_rate = 0.0
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 1e-4:
                dyaw = yaw - self.last_yaw
                dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
                yaw_rate = dyaw / dt

        self.last_yaw = yaw
        self.last_time = now

        # Publish debug state
        out = (
            f"x={x:.2f} y={y:.2f} "
            f"yaw={yaw:.3f} rad "
            f"v={v:.2f} m/s "
            f"yaw_rate={yaw_rate:.3f} rad/s"
        )

        msg_out = String()
        msg_out.data = out
        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = FsdsStateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


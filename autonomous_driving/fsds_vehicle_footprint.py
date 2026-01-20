#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class FSDSVehicleFootprint(Node):
    def __init__(self):
        super().__init__("fsds_vehicle_footprint")

        # ---- vehicle dimensions (meters) ----
        self.length = 2.8
        self.width = 1.5
        self.rear_axle_offset = 0.8

        self.sub_odom = self.create_subscription(
            Odometry,
            "/testing_only/odom",
            self.odom_cb,
            10
        )

        self.pub_marker = self.create_publisher(
            Marker,
            "/vehicle_debug",
            10
        )

        self.get_logger().info("✅ Vehicle footprint + heading marker running")

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        self.publish_footprint(pos.x, pos.y, yaw)
        self.publish_heading_arrow(pos.x, pos.y, yaw)
        self.publish_rear_axle(pos.x, pos.y, yaw)

    def publish_footprint(self, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = "fsds/FSCar"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle_footprint"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        half_l = self.length / 2.0
        half_w = self.width / 2.0

        corners = [
            ( half_l,  half_w),
            ( half_l, -half_w),
            (-half_l, -half_w),
            (-half_l,  half_w),
            ( half_l,  half_w),
        ]

        pts = []
        for cx, cy in corners:
            rx = cx * math.cos(yaw) - cy * math.sin(yaw)
            ry = cx * math.sin(yaw) + cy * math.cos(yaw)
            p = Point()
            p.x = x + rx
            p.y = y + ry
            p.z = 0.05
            pts.append(p)

        marker.points = pts
        self.pub_marker.publish(marker)

    def publish_heading_arrow(self, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = "fsds/FSCar"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle_heading"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        start = Point()
        start.x = x
        start.y = y
        start.z = 0.1

        end = Point()
        end.x = x + math.cos(yaw)
        end.y = y + math.sin(yaw)
        end.z = 0.1

        marker.points = [start, end]
        self.pub_marker.publish(marker)

    def publish_rear_axle(self, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = "fsds/FSCar"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rear_axle"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.position.x = x - self.rear_axle_offset * math.cos(yaw)
        marker.pose.position.y = y - self.rear_axle_offset * math.sin(yaw)
        marker.pose.position.z = 0.1

        marker.pose.orientation.w = 1.0
        self.pub_marker.publish(marker)


def main():
    rclpy.init()
    node = FSDSVehicleFootprint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


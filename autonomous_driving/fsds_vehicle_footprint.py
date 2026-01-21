#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class FSDSVehicleFootprint(Node):
    def __init__(self):
        super().__init__('fsds_vehicle_footprint')

        self.pub = self.create_publisher(Marker, '/vehicle_footprint', 10)
        self.timer = self.create_timer(0.05, self.publish_marker)

        self.get_logger().info("✅ Vehicle footprint + heading marker running (ego-centric)")

    def publish_marker(self):
        marker = Marker()

        # ===============================
        # Frame: EGO FRAME (CRITICAL)
        # ===============================
        marker.header.frame_id = "fsds/FSCar"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # ===============================
        # VEHICLE DIMENSIONS (FSDS approx)
        # ===============================
        length = 1.6   # meters
        width = 0.9

        l = length / 2.0
        w = width / 2.0

        # ===============================
        # FOOTPRINT (RECTANGLE)
        # ===============================
        corners = [
            Point(x=+l, y=+w, z=0.0),
            Point(x=+l, y=-w, z=0.0),

            Point(x=+l, y=-w, z=0.0),
            Point(x=-l, y=-w, z=0.0),

            Point(x=-l, y=-w, z=0.0),
            Point(x=-l, y=+w, z=0.0),

            Point(x=-l, y=+w, z=0.0),
            Point(x=+l, y=+w, z=0.0),
        ]

        # ===============================
        # HEADING ARROW (FORWARD = +X)
        # ===============================
        heading = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=+l + 0.5, y=0.0, z=0.0),
        ]

        marker.points = corners + heading

        # ===============================
        # STYLE
        # ===============================
        marker.scale.x = 0.05  # line thickness

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # ===============================
        # POSE MUST BE ZERO (EGO)
        # ===============================
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        self.pub.publish(marker)


def main():
    rclpy.init()
    node = FSDSVehicleFootprint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


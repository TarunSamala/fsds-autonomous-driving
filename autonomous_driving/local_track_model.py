#!/usr/bin/env python3

import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class LocalTrackModel(Node):
    """
    Phase 3 – Local Track Model

    - Consumes left/right cone markers
    - Maintains temporal stability
    - Generates forward local centerline
    - NO control logic
    """

    def __init__(self):
        super().__init__('local_track_model')

        # Subscribers
        self.left_sub = self.create_subscription(
            MarkerArray, '/left_cones', self.left_cb, 10)
        self.right_sub = self.create_subscription(
            MarkerArray, '/right_cones', self.right_cb, 10)

        # Publisher
        self.center_pub = self.create_publisher(
            MarkerArray, '/local_centerline', 10)

        # Rolling buffers
        self.left_buffer = deque(maxlen=10)
        self.right_buffer = deque(maxlen=10)

        # Parameters
        self.max_forward_dist = 20.0  # meters

        self.get_logger().info("✅ Local Track Model started")

    # ------------------------------------------------------------

    def left_cb(self, msg: MarkerArray):
        self.left_buffer.append(self.extract_points(msg))

    def right_cb(self, msg: MarkerArray):
        self.right_buffer.append(self.extract_points(msg))

    # ------------------------------------------------------------

    def extract_points(self, msg: MarkerArray):
        pts = []
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                continue
            pts.append([m.pose.position.x, m.pose.position.y])
        return np.array(pts)

    # ------------------------------------------------------------

    def compute_centerline(self):
        if not self.left_buffer or not self.right_buffer:
            return []

        left = np.vstack(self.left_buffer)
        right = np.vstack(self.right_buffer)

        if left.size == 0 or right.size == 0:
            return []

        # Forward only
        left = left[left[:, 0] > 0.0]
        right = right[right[:, 0] > 0.0]

        if len(left) == 0 or len(right) == 0:
            return []

        # Sort by X
        left = left[np.argsort(left[:, 0])]
        right = right[np.argsort(right[:, 0])]

        n = min(len(left), len(right))
        centers = []

        for i in range(n):
            c = 0.5 * (left[i] + right[i])
            if 0.0 < c[0] < self.max_forward_dist:
                centers.append(c)

        return centers

    # ------------------------------------------------------------

    def publish_centerline(self):
        centers = self.compute_centerline()
        if not centers:
            return

        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear previous markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, c in enumerate(centers):
            m = Marker()
            m.header.frame_id = 'fsds/FSCar'
            m.header.stamp = now
            m.ns = 'centerline'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = 0.2

            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3

            m.color.b = 1.0
            m.color.a = 1.0

            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.2 * 1e9)

            arr.markers.append(m)

        self.center_pub.publish(arr)

    # ------------------------------------------------------------

    def timer_cb(self):
        self.publish_centerline()


def main(args=None):
    rclpy.init(args=args)
    node = LocalTrackModel()
    node.create_timer(0.1, node.timer_cb)  # 10 Hz
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


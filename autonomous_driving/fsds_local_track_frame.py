#!/usr/bin/env python3
"""
FSDS Local Track Frame (Vehicle Frame Only)

Builds:
- Track corridor polygon (red)
- Ordered left/right boundaries
- Forward-only Frenet-aligned local centerline (green)

This node is the geometric contract between perception and control.
"""

import rclpy
from rclpy.node import Node
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class FSDSLocalTrackFrame(Node):
    def __init__(self):
        super().__init__('fsds_local_track_frame')

        # ---------------- Parameters ----------------
        self.min_x = 0.5
        self.max_x = 10.0
        self.max_pairs = 30
        self.segment_len = 0.35

        # ---------------- State ----------------
        self.left = []
        self.right = []

        # ---------------- Subscriptions ----------------
        self.create_subscription(
            MarkerArray, '/left_cones', self.left_cb, 10
        )
        self.create_subscription(
            MarkerArray, '/right_cones', self.right_cb, 10
        )

        # ---------------- Publishers ----------------
        self.pub_poly = self.create_publisher(MarkerArray, '/track_polygon', 1)
        self.pub_left = self.create_publisher(MarkerArray, '/ltm_debug/left_boundary', 1)
        self.pub_right = self.create_publisher(MarkerArray, '/ltm_debug/right_boundary', 1)
        self.pub_center = self.create_publisher(MarkerArray, '/local_centerline_frenet', 1)

        self.timer = self.create_timer(0.1, self.process)

        self.get_logger().info("✅ Local Track Frame running (vehicle frame)")

    # =====================================================
    def left_cb(self, msg):
        self.left = [(m.pose.position.x, m.pose.position.y)
                     for m in msg.markers if m.pose.position.x > 0]

    def right_cb(self, msg):
        self.right = [(m.pose.position.x, m.pose.position.y)
                      for m in msg.markers if m.pose.position.x > 0]

    # =====================================================
    def process(self):
        if len(self.left) < 2 or len(self.right) < 2:
            return

        # ---- Order boundaries by forward x ----
        L = sorted(self.left, key=lambda p: p[0])[:self.max_pairs]
        R = sorted(self.right, key=lambda p: p[0])[:self.max_pairs]

        n = min(len(L), len(R))
        L, R = L[:n], R[:n]

        # ---- Corridor + boundaries ----
        poly = MarkerArray()
        left_m = MarkerArray()
        right_m = MarkerArray()

        for i in range(n):
            # Left boundary
            left_m.markers.append(self.point_marker(
                L[i], i, 'left', (0.1, 0.3, 1.0)
            ))
            # Right boundary
            right_m.markers.append(self.point_marker(
                R[i], i, 'right', (1.0, 0.9, 0.1)
            ))

            # Red corridor rung
            poly.markers.append(self.line_marker(
                L[i], R[i], i, 'poly'
            ))

        self.pub_left.publish(left_m)
        self.pub_right.publish(right_m)
        self.pub_poly.publish(poly)

        # ---- Centerline midpoints ----
        mids = []
        for i in range(n):
            mx = 0.5 * (L[i][0] + R[i][0])
            my = 0.5 * (L[i][1] + R[i][1])
            if self.min_x < mx < self.max_x:
                mids.append((mx, my))

        if len(mids) < 2:
            return

        # ---- Arc-length ordering ----
        ordered = [mids[0]]
        for p in mids[1:]:
            if math.hypot(p[0] - ordered[-1][0],
                          p[1] - ordered[-1][1]) > 0.2:
                ordered.append(p)

        # ---- Frenet tangents ----
        center = MarkerArray()
        for i in range(len(ordered) - 1):
            x1, y1 = ordered[i]
            x2, y2 = ordered[i + 1]

            yaw = math.atan2(y2 - y1, x2 - x1)
            dx = self.segment_len * math.cos(yaw)
            dy = self.segment_len * math.sin(yaw)

            center.markers.append(self.line_marker(
                (x1, y1), (x1 + dx, y1 + dy), i, 'center',
                color=(0.1, 1.0, 0.1)
            ))

        self.pub_center.publish(center)

    # =====================================================
    def point_marker(self, p, idx, ns, color):
        m = Marker()
        m.header.frame_id = 'fsds/FSCar'
        m.ns = ns
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = p[0]
        m.pose.position.y = p[1]
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return m

    def line_marker(self, p1, p2, idx, ns, color=(1.0, 0.1, 0.1)):
        m = Marker()
        m.header.frame_id = 'fsds/FSCar'
        m.ns = ns
        m.id = idx
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0

        m.points = [Point(x=p1[0], y=p1[1], z=0.05),
                    Point(x=p2[0], y=p2[1], z=0.05)]
        return m


def main(args=None):
    rclpy.init(args=args)
    node = FSDSLocalTrackFrame()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


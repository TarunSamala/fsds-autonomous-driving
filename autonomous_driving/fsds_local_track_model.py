#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class LocalTrackModel(Node):
    def __init__(self):
        super().__init__('fsds_local_track_model')

        self.left_sub = self.create_subscription(
            MarkerArray, '/left_cones', self.left_cb, 10)
        self.right_sub = self.create_subscription(
            MarkerArray, '/right_cones', self.right_cb, 10)

        self.center_pub = self.create_publisher(
            MarkerArray, '/local_centerline', 10)
        self.left_dbg = self.create_publisher(
            MarkerArray, '/ltm_debug/left_track', 10)
        self.right_dbg = self.create_publisher(
            MarkerArray, '/ltm_debug/right_track', 10)

        self.left_track = deque(maxlen=50)
        self.right_track = deque(maxlen=50)

        self.assoc_dist = 0.6
        self.alpha = 0.3  # EMA smoothing

        self.timer = self.create_timer(0.1, self.publish_centerline)

        self.get_logger().info("✅ Local Track Model started")

    # ---------- Callbacks ----------

    def left_cb(self, msg):
        self.update_track(msg, self.left_track)

    def right_cb(self, msg):
        self.update_track(msg, self.right_track)

    # ---------- Core logic ----------

    def update_track(self, msg, track):
        now = time.time()
        for m in msg.markers:
            x = m.pose.position.x
            y = m.pose.position.y

            if x <= 0.0:
                continue

            matched = False
            for i, (tx, ty, _) in enumerate(track):
                d = math.hypot(x - tx, y - ty)
                if d < self.assoc_dist:
                    nx = self.alpha * x + (1 - self.alpha) * tx
                    ny = self.alpha * y + (1 - self.alpha) * ty
                    track[i] = (nx, ny, now)
                    matched = True
                    break

            if not matched:
                track.append((x, y, now))

    def publish_centerline(self):
        if not self.left_track or not self.right_track:
            return

        left = sorted(self.left_track, key=lambda p: p[0])
        right = sorted(self.right_track, key=lambda p: p[0])

        n = min(len(left), len(right))
        now = self.get_clock().now().to_msg()

        center_arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        center_arr.markers.append(clear)

        for i in range(n):
            lx, ly, _ = left[i]
            rx, ry, _ = right[i]

            cx = 0.5 * (lx + rx)
            cy = 0.5 * (ly + ry)

            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.ns = "centerline"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0

            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15

            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0

            center_arr.markers.append(m)

        self.center_pub.publish(center_arr)


def main(args=None):
    rclpy.init(args=args)
    node = LocalTrackModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


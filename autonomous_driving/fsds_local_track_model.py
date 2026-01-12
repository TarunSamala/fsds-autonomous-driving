#!/usr/bin/env python3
"""
FSDS Local Track Model (Phase 3)

Converts transient cone detections into:
- persistent left/right tracks
- a stable, forward-only local centerline

NO control logic.
NO global mapping.
Pure geometry.
"""

import time
import math
from collections import deque

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class LocalTrackModel(Node):
    def __init__(self):
        super().__init__('fsds_local_track_model')

        # ---------------- Subscriptions ----------------
        self.left_sub = self.create_subscription(
            MarkerArray, '/left_cones', self.left_cb, 10)
        self.right_sub = self.create_subscription(
            MarkerArray, '/right_cones', self.right_cb, 10)

        # ---------------- Publishers ----------------
        self.center_pub = self.create_publisher(
            MarkerArray, '/local_centerline', 10)
        self.left_dbg = self.create_publisher(
            MarkerArray, '/ltm_debug/left_track', 10)
        self.right_dbg = self.create_publisher(
            MarkerArray, '/ltm_debug/right_track', 10)

        # ---------------- State ----------------
        # (x, y, timestamp)
        self.left_track = deque(maxlen=50)
        self.right_track = deque(maxlen=50)

        self.assoc_dist = 0.6        # meters
        self.alpha = 0.3             # EMA smoothing
        self.track_timeout = 1.0     # seconds

        self.timer = self.create_timer(0.1, self.publish_all)

        self.get_logger().info("✅ FSDS Local Track Model running")

    # =========================================================
    # Callbacks
    # =========================================================

    def left_cb(self, msg):
        self.update_track(msg, self.left_track)

    def right_cb(self, msg):
        self.update_track(msg, self.right_track)

    # =========================================================
    # Core logic
    # =========================================================

    def update_track(self, msg, track):
        now = time.time()

        for m in msg.markers:
            x = m.pose.position.x
            y = m.pose.position.y

            # Forward-only
            if x <= 0.0:
                continue

            matched = False
            for i, (tx, ty, _) in enumerate(track):
                d = math.hypot(x - tx, y - ty)
                if d < self.assoc_dist:
                    nx = self.alpha * x + (1.0 - self.alpha) * tx
                    ny = self.alpha * y + (1.0 - self.alpha) * ty
                    track[i] = (nx, ny, now)
                    matched = True
                    break

            if not matched:
                track.append((x, y, now))

    def prune_tracks(self):
        now = time.time()
        self.left_track = deque(
            [p for p in self.left_track if now - p[2] < self.track_timeout],
            maxlen=50
        )
        self.right_track = deque(
            [p for p in self.right_track if now - p[2] < self.track_timeout],
            maxlen=50
        )

    # =========================================================
    # Publishing
    # =========================================================

    def publish_all(self):
        self.prune_tracks()

        # Publish debug tracks
        self.publish_track(
            self.left_track, self.left_dbg,
            ns="left_track", color=(0.0, 0.0, 1.0)
        )
        self.publish_track(
            self.right_track, self.right_dbg,
            ns="right_track", color=(1.0, 1.0, 0.0)
        )

        # Publish centerline
        self.publish_centerline()

    def publish_track(self, track, pub, ns, color):
        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        now = self.get_clock().now().to_msg()

        for i, (x, y, _) in enumerate(track):
            if x <= 0.0:
                continue

            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.ns = ns
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0

            m.scale.x = 0.30
            m.scale.y = 0.30
            m.scale.z = 0.30

            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0

            arr.markers.append(m)

        pub.publish(arr)

    def publish_centerline(self):
        if not self.left_track or not self.right_track:
            return

        left = sorted(self.left_track, key=lambda p: p[0])
        right = sorted(self.right_track, key=lambda p: p[0])

        n = min(len(left), len(right))
        if n == 0:
            return

        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        now = self.get_clock().now().to_msg()

        for i in range(n):
            lx, ly, _ = left[i]
            rx, ry, _ = right[i]

            cx = 0.5 * (lx + rx)
            cy = 0.5 * (ly + ry)

            if cx <= 0.0:
                continue

            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.ns = "centerline"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.12
            m.pose.orientation.w = 1.0

            # BIG + RED for debugging clarity
            m.scale.x = 0.45
            m.scale.y = 0.45
            m.scale.z = 0.45

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0

            arr.markers.append(m)

        self.center_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LocalTrackModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


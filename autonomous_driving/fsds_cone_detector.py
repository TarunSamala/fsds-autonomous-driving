#!/usr/bin/env python3

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class FsdsConeDetector(Node):
    def __init__(self):
        super().__init__('fsds_cone_detector')

        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/Lidar1',
            self.lidar_cb,
            10
        )

        self.left_pub = self.create_publisher(
            MarkerArray,
            '/left_cones',
            10
        )

        self.right_pub = self.create_publisher(
            MarkerArray,
            '/right_cones',
            10
        )

        self.get_logger().info("✅ FSDS Cone Detector started")

        # Clustering params (conservative)
        self.cluster_radius = 0.25   # meters
        self.min_cluster_pts = 6

    def lidar_cb(self, msg: PointCloud2):
        pts = self.pointcloud_to_xyz(msg)
        if pts.size == 0:
            return

        # --- Filter geometry ---
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        r = np.sqrt(x**2 + y**2)

        mask = (
            (z > 0.05) & (z < 0.6) &
            (r > 0.5) & (r < 20.0)
        )

        pts = pts[mask]
        if pts.shape[0] < self.min_cluster_pts:
            return

        # --- Cluster ---
        clusters = self.euclidean_clustering(pts)

        left_centroids = []
        right_centroids = []

        for c in clusters:
            centroid = np.mean(c, axis=0)

            # FSDS semantic:
            #   Y > 0 → LEFT (blue)
            #   Y < 0 → RIGHT (yellow)
            if centroid[1] > 0.0:
                left_centroids.append(centroid)
            else:
                right_centroids.append(centroid)

        self.left_pub.publish(self.make_markers(left_centroids, color='blue'))
        self.right_pub.publish(self.make_markers(right_centroids, color='yellow'))

    # ---------------- Helpers ----------------

    def pointcloud_to_xyz(self, msg: PointCloud2):
        if msg.point_step < 12:
            return np.empty((0, 3), dtype=np.float32)

        n = len(msg.data) // msg.point_step
        pts = np.zeros((n, 3), dtype=np.float32)

        for i in range(n):
            base = i * msg.point_step
            try:
                x = struct.unpack_from('f', msg.data, base + 0)[0]
                y = struct.unpack_from('f', msg.data, base + 4)[0]
                z = struct.unpack_from('f', msg.data, base + 8)[0]
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                    pts[i] = (x, y, z)
            except Exception:
                pass

        return pts

    def euclidean_clustering(self, pts):
        clusters = []
        used = np.zeros(len(pts), dtype=bool)

        for i in range(len(pts)):
            if used[i]:
                continue

            seed = [i]
            used[i] = True
            cluster = []

            while seed:
                idx = seed.pop()
                cluster.append(pts[idx])

                dists = np.linalg.norm(pts - pts[idx], axis=1)
                neighbors = np.where((dists < self.cluster_radius) & (~used))[0]

                for n in neighbors:
                    used[n] = True
                    seed.append(n)

            if len(cluster) >= self.min_cluster_pts:
                clusters.append(np.array(cluster))

        return clusters

    def make_markers(self, centroids, color):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, c in enumerate(centroids):
            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0

            m.scale.x = 0.23
            m.scale.y = 0.23
            m.scale.z = 0.4

            if color == 'blue':
                m.color.b = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 1.0

            m.color.a = 0.9
            arr.markers.append(m)

        return arr


def main(args=None):
    rclpy.init(args=args)
    node = FsdsConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


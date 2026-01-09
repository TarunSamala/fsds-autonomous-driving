#!/usr/bin/env python3

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class FsdsConeDetector(Node):
    """
    Phase 2 – Perception (FSDS)

    - Consumes LiDAR PointCloud2
    - Clusters cone points
    - Publishes cone centroids as MarkerArray
    - NO control, NO mapping, NO odometry coupling
    """

    def __init__(self):
        super().__init__('fsds_cone_detector')

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/Lidar1',
            self.lidar_cb,
            10
        )

        # Publishers
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

        # Clustering parameters (FSDS-proven)
        self.cluster_radius = 0.20   # meters
        self.min_cluster_pts = 5

        self.get_logger().info("✅ FSDS Cone Detector started (Phase 2)")

    # ------------------------------------------------------------

    def lidar_cb(self, msg: PointCloud2):
        pts = self.pointcloud2_to_xyz(msg)
        if pts.size == 0:
            return

        # --- FSDS-CORRECT FILTERING ---
        z = pts[:, 2]
        r = np.linalg.norm(pts[:, :2], axis=1)

        mask = (
            (z > -0.3) & (z < 0.3) &
            (r > 0.5) & (r < 15.0)
        )

        pts = pts[mask]

        self.get_logger().debug(
            f"LiDAR pts total={len(msg.data)//msg.point_step}, after filter={pts.shape[0]}"
        )

        if pts.shape[0] < self.min_cluster_pts:
            return

        # --- CLUSTER ---
        clusters = self.euclidean_clustering(pts)

        left_centroids = []
        right_centroids = []

        for c in clusters:
            centroid = np.mean(c, axis=0)

            # FSDS semantic convention
            #   +Y → LEFT (blue)
            #   -Y → RIGHT (yellow)
            if centroid[1] > 0.0:
                left_centroids.append(centroid)
            else:
                right_centroids.append(centroid)

        # Publish
        self.left_pub.publish(self.make_markers(left_centroids, "blue"))
        self.right_pub.publish(self.make_markers(right_centroids, "yellow"))

    # ------------------------------------------------------------

    def pointcloud2_to_xyz(self, msg: PointCloud2):
        """Convert PointCloud2 → (N,3) numpy array"""
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

    # ------------------------------------------------------------

    def euclidean_clustering(self, pts):
        """Simple Euclidean clustering (no sklearn)"""
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
                neighbors = np.where(
                    (dists < self.cluster_radius) & (~used)
                )[0]

                for n in neighbors:
                    used[n] = True
                    seed.append(n)

            if len(cluster) >= self.min_cluster_pts:
                clusters.append(np.array(cluster))

        return clusters

    # ------------------------------------------------------------

    def make_markers(self, centroids, color):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear old markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, c in enumerate(centroids):
            m = Marker()
            m.header.frame_id = 'fsds/Lidar1'   # IMPORTANT (TF-safe)
            m.header.stamp = now
            m.ns = color
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

            if color == "blue":
                m.color.b = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 1.0

            m.color.a = 1.0

            # Prevent RViz accumulation
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.2 * 1e9)

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


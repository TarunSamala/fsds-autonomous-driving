import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import struct
import numpy as np


class SimpleDBSCAN:
    """Minimal DBSCAN implementation using NumPy only (no sklearn)."""

    def __init__(self, eps=0.15, min_samples=5):
        self.eps = eps
        self.min_samples = min_samples

    def fit(self, points: np.ndarray):
        n = points.shape[0]
        labels = -np.ones(n, dtype=int)  # -1 = noise
        visited = np.zeros(n, dtype=bool)
        cluster_id = 0

        # Precompute distance matrix (fine for a few thousand points)
        dists = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)

        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True

            neighbors = np.where(dists[i] <= self.eps)[0]
            if neighbors.size < self.min_samples:
                labels[i] = -1  # noise
            else:
                # Start new cluster
                labels[i] = cluster_id
                seeds = list(neighbors[neighbors != i])

                while seeds:
                    j = seeds.pop()
                    if not visited[j]:
                        visited[j] = True
                        j_neighbors = np.where(dists[j] <= self.eps)[0]
                        if j_neighbors.size >= self.min_samples:
                            # Add new neighbors to seeds
                            for k in j_neighbors:
                                if k not in seeds:
                                    seeds.append(k)
                    if labels[j] == -1:
                        labels[j] = cluster_id

                cluster_id += 1

        return labels


class LidarConeDetector(Node):
    def __init__(self):
        super().__init__('lidar_cone_detector')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/Lidar1',
            self.lidar_callback,
            10)

        self.publisher = self.create_publisher(
            MarkerArray,
            '/detected_cones',
            10)

        # Clustering params – we can tune later
        self.eps = 0.20
        self.min_samples = 5
        self.get_logger().info('LiDAR cone detector (NumPy-only) started')

    def lidar_callback(self, msg: PointCloud2):
        points = self.pointcloud2_to_xyz(msg)
        if points.size == 0:
            return

        # Filter by height and distance
        z = points[:, 2]
        dist = np.linalg.norm(points[:, :3], axis=1)
        mask = (z > -0.3) & (z < 0.3) & (dist > 0.5) & (dist < 15.0)
        points = points[mask]

        if points.shape[0] < self.min_samples:
            return

        db = SimpleDBSCAN(eps=self.eps, min_samples=self.min_samples)
        labels = db.fit(points[:, :3])

        centers = []
        for label in np.unique(labels):
            if label == -1:
                continue
            cluster_pts = points[labels == label]
            if cluster_pts.shape[0] == 0:
                continue
            centers.append(cluster_pts[:, :3].mean(axis=0))

        self.publish_markers(centers)
        self.get_logger().info(f'Detected {len(centers)} cones')

    def pointcloud2_to_xyz(self, msg: PointCloud2):
        """Convert PointCloud2 to (N,3) float32 array."""
        if msg.point_step < 12:
            return np.empty((0, 3), dtype=np.float32)

        n_points = len(msg.data) // msg.point_step
        pts = np.zeros((n_points, 3), dtype=np.float32)

        offset_x = 0
        offset_y = 4
        offset_z = 8

        for i in range(n_points):
            base = i * msg.point_step
            x = struct.unpack_from('f', msg.data, base + offset_x)[0]
            y = struct.unpack_from('f', msg.data, base + offset_y)[0]
            z = struct.unpack_from('f', msg.data, base + offset_z)[0]
            pts[i] = (x, y, z)

        return pts

    def publish_markers(self, centers):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, c in enumerate(centers):
            m = Marker()
            m.header.frame_id = 'fsds/Lidar1'
            m.header.stamp = now
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.3
            m.color.r = 0.5
            m.color.g = 0.5
            m.color.b = 0.5
            m.color.a = 0.9
            markers.markers.append(m)

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = LidarConeDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


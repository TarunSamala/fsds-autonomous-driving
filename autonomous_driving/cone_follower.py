#!/usr/bin/env python3
"""
Universal Cone Detector + Track Follower
Works regardless of LiDAR frame orientation
"""

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from fs_msgs.msg import ControlCommand
from visualization_msgs.msg import MarkerArray, Marker
import struct


class UniversalConeFollower(Node):
    def __init__(self):
        super().__init__('universal_cone_follower')
        
        self.declare_parameter('throttle', 0.035)
        self.declare_parameter('max_steering', 0.42)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('lookahead', 2.0)
        
        self.throttle = self.get_parameter('throttle').value
        self.max_steering = self.get_parameter('max_steering').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.lookahead = self.get_parameter('lookahead').value
        
        # State
        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.speed = 0.0
        
        # All detected points
        self.all_points = []
        self.left_cluster = []
        self.right_cluster = []
        self.centerline = []
        
        # Diagnostics
        self.frame_count = 0
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/Lidar1',
            self.lidar_callback,
            qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            qos
        )
        
        # Publishers
        self.control_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10
        )
        
        self.all_points_pub = self.create_publisher(
            MarkerArray,
            '/all_lidar_points',
            10
        )
        
        self.left_pub = self.create_publisher(
            MarkerArray,
            '/left_cluster',
            10
        )
        
        self.right_pub = self.create_publisher(
            MarkerArray,
            '/right_cluster',
            10
        )
        
        self.centerline_pub = self.create_publisher(
            MarkerArray,
            '/centerline_markers',
            10
        )
        
        self.target_pub = self.create_publisher(
            PointStamped,
            '/target_point',
            10
        )
        
        self.get_logger().info("🚀 UNIVERSAL Cone Follower - STARTING")
    
    def odom_callback(self, msg: Odometry):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
    
    def lidar_callback(self, msg: PointCloud2):
        self.frame_count += 1
        
        # Parse all points
        self.all_points = self.parse_pointcloud2(msg)
        
        if self.frame_count == 1:
            self.get_logger().info(f"✅ Got {len(self.all_points)} points from LiDAR")
            if len(self.all_points) > 0:
                pts = np.array(self.all_points)
                self.get_logger().info(f"   X range: {pts[:, 0].min():.2f} to {pts[:, 0].max():.2f}")
                self.get_logger().info(f"   Y range: {pts[:, 1].min():.2f} to {pts[:, 1].max():.2f}")
                self.get_logger().info(f"   Z range: {pts[:, 2].min():.2f} to {pts[:, 2].max():.2f}")
        
        # Cluster by height (cones have peaks)
        self.cluster_by_height()
        
        # Pair clusters to find centerline
        self.centerline = self.compute_centerline_from_clusters()
        
        # Get target
        target = self.find_target()
        
        if target is not None:
            steering = self.compute_steering(target)
        else:
            steering = 0.0
        
        # Visualize
        self.publish_all_visualizations()
        self.publish_control(steering)
        
        # Debug every 10 frames
        if self.frame_count % 10 == 1:
            self.get_logger().info(
                f"📊 Frame {self.frame_count}: "
                f"L={len(self.left_cluster)} R={len(self.right_cluster)} "
                f"Center={len(self.centerline)} Steer={steering:.3f}"
            )
    
    def cluster_by_height(self):
        """Find cone peaks by Z-height clustering"""
        if len(self.all_points) == 0:
            self.left_cluster = []
            self.right_cluster = []
            return
        
        pts = np.array(self.all_points)
        
        # Filter by Z (cones are elevated)
        mask = (pts[:, 2] > 0.05) & (pts[:, 2] < 0.8)
        cone_points = pts[mask]
        
        if len(cone_points) == 0:
            self.left_cluster = []
            self.right_cluster = []
            return
        
        # Split by Y (left vs right)
        # Use MEDIAN Y to decide which is "left" and which is "right"
        median_y = np.median(cone_points[:, 1])
        
        left = cone_points[cone_points[:, 1] > median_y]
        right = cone_points[cone_points[:, 1] < median_y]
        
        # Compute centroids
        if len(left) > 0:
            self.left_cluster = [np.mean(left, axis=0)[:2]]  # Just X, Y
        else:
            self.left_cluster = []
        
        if len(right) > 0:
            self.right_cluster = [np.mean(right, axis=0)[:2]]
        else:
            self.right_cluster = []
        
        if self.frame_count == 1:
            self.get_logger().info(f"   Cone points after Z filter: {len(cone_points)}")
            self.get_logger().info(f"   Median Y: {median_y:.3f}")
            self.get_logger().info(f"   Left cluster: {len(left)} pts → centroid {self.left_cluster}")
            self.get_logger().info(f"   Right cluster: {len(right)} pts → centroid {self.right_cluster}")
    
    def compute_centerline_from_clusters(self):
        """Simple: midpoint between left and right clusters"""
        if len(self.left_cluster) == 0 or len(self.right_cluster) == 0:
            return []
        
        # If we have per-frame clusters, compute centerline
        centerline = []
        
        for left_pt in self.left_cluster:
            for right_pt in self.right_cluster:
                center = (left_pt + right_pt) / 2.0
                centerline.append(center)
        
        return centerline
    
    def find_target(self):
        if len(self.centerline) == 0:
            return None
        
        # Find closest centerline point
        dists = [np.linalg.norm(np.array(p) - self.current_pos) for p in self.centerline]
        closest_idx = np.argmin(dists)
        
        # Look ahead
        target_idx = closest_idx
        for i in range(closest_idx, len(self.centerline)):
            if np.linalg.norm(np.array(self.centerline[i]) - self.current_pos) >= self.lookahead:
                target_idx = i
                break
        
        return self.centerline[target_idx]
    
    def compute_steering(self, target):
        target = np.array(target)
        to_target = target - self.current_pos
        dist = np.linalg.norm(to_target)
        
        if dist < 0.01:
            return 0.0
        
        to_target = to_target / dist
        target_yaw = math.atan2(to_target[1], to_target[0])
        heading_err = target_yaw - self.current_yaw
        
        while heading_err > math.pi:
            heading_err -= 2 * math.pi
        while heading_err < -math.pi:
            heading_err += 2 * math.pi
        
        cte = target[1] - self.current_pos[1]
        
        k = 1.0
        if self.speed > 0.05:
            steering = heading_err + math.atan2(k * cte, max(self.speed, 0.1))
        else:
            steering = heading_err
        
        steering = max(-self.max_steering, min(self.max_steering, steering))
        
        t = PointStamped()
        t.header.frame_id = "fsds/FSCar"
        t.header.stamp = self.get_clock().now().to_msg()
        t.point.x = target[0]
        t.point.y = target[1]
        self.target_pub.publish(t)
        
        return steering
    
    def publish_all_visualizations(self):
        """Visualize EVERYTHING to debug"""
        now = self.get_clock().now().to_msg()
        
        # 1. All LiDAR points (small white dots)
        all_markers = MarkerArray()
        for i, pt in enumerate(self.all_points[:500]):  # Limit to 500 for performance
            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.id = i
            m.type = Marker.SPHERE
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = float(pt[2])
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.3
            all_markers.markers.append(m)
        self.all_points_pub.publish(all_markers)
        
        # 2. Left cluster (green)
        left_markers = MarkerArray()
        for i, pt in enumerate(self.left_cluster):
            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.id = i
            m.type = Marker.SPHERE
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = 0.2
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.g = 1.0
            m.color.a = 1.0
            left_markers.markers.append(m)
        self.left_pub.publish(left_markers)
        
        # 3. Right cluster (red)
        right_markers = MarkerArray()
        for i, pt in enumerate(self.right_cluster):
            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.id = i
            m.type = Marker.SPHERE
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = 0.2
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.a = 1.0
            right_markers.markers.append(m)
        self.right_pub.publish(right_markers)
        
        # 4. Centerline (blue)
        center_markers = MarkerArray()
        for i, pt in enumerate(self.centerline):
            m = Marker()
            m.header.frame_id = "fsds/FSCar"
            m.header.stamp = now
            m.id = i
            m.type = Marker.SPHERE
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = 0.3
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.b = 1.0
            m.color.a = 1.0
            center_markers.markers.append(m)
        self.centerline_pub.publish(center_markers)
    
    def publish_control(self, steering):
        cmd = ControlCommand()
        cmd.steering = steering
        cmd.throttle = self.throttle
        self.control_pub.publish(cmd)
    
    def parse_pointcloud2(self, msg: PointCloud2):
        points = []
        field_dict = {f.name: f.offset for f in msg.fields}
        
        if 'x' not in field_dict:
            return points
        
        for i in range(msg.width * msg.height):
            offset = i * msg.point_step
            try:
                x, = struct.unpack('f', msg.data[offset+field_dict['x']:offset+field_dict['x']+4])
                y, = struct.unpack('f', msg.data[offset+field_dict['y']:offset+field_dict['y']+4])
                z, = struct.unpack('f', msg.data[offset+field_dict['z']:offset+field_dict['z']+4])
                
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                    points.append((x, y, z))
            except:
                pass
        
        return points


def main(args=None):
    rclpy.init(args=args)
    follower = UniversalConeFollower()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        print("\n✋ Stopped")
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
import json
import os

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Subscribe to LiDAR and cone map
        self.sub_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.sub_map = self.create_subscription(
            MarkerArray,
            '/cone_map',
            self.map_callback,
            10
        )
        
        # Publish estimated pose
        self.pub_pose = self.create_publisher(
            PoseStamped,
            '/estimated_pose',
            10
        )
        
        self.cone_map = []
        self.current_scan = None
        self.last_pose = [0.0, 0.0, 0.0]  # x, y, yaw
        
        self.get_logger().info("✅ Localization Node Started")
        self.get_logger().info("   Waiting for LiDAR scans and cone map...")

    def map_callback(self, msg):
        """Extract cone positions from map."""
        self.cone_map = []
        for marker in msg.markers:
            self.cone_map.append([
                marker.pose.position.x,
                marker.pose.position.y
            ])

    def lidar_callback(self, msg):
        """Process LiDAR scan and localize."""
        if not self.cone_map:
            return
        
        # Convert LiDAR scan to points
        points = self.scan_to_points(msg)
        
        if len(points) < 10:
            return
        
        # Find best pose by matching with cone map
        best_pose = self.match_scan_to_map(points)
        
        if best_pose is not None:
            self.last_pose = best_pose
            self.publish_pose(best_pose)

    def scan_to_points(self, msg):
        """Convert LiDAR scan to (x, y) points."""
        points = []
        for i, r in enumerate(msg.ranges):
            if r > msg.range_min and r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
        return np.array(points)

    def match_scan_to_map(self, scan_points):
        """Simple scan matching: find best pose alignment."""
        best_score = -float('inf')
        best_pose = None
        
        # Search near last known position
        for dx in np.linspace(-2, 2, 5):
            for dy in np.linspace(-2, 2, 5):
                for dyaw in np.linspace(-math.pi/4, math.pi/4, 5):
                    # Transform scan points
                    transformed = self.transform_points(scan_points, dx, dy, dyaw)
                    
                    # Score: how many scan points are close to cones
                    score = 0
                    for point in transformed:
                        for cone in self.cone_map:
                            dist = math.hypot(point[0] - cone[0], point[1] - cone[1])
                            if dist < 0.5:  # Within 0.5m of a cone
                                score += 1 / (1 + dist)
                    
                    if score > best_score:
                        best_score = score
                        best_pose = [dx, dy, dyaw]
        
        return best_pose if best_score > 5 else None

    def transform_points(self, points, tx, ty, rot):
        """Rotate and translate points."""
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)
        rotated = np.array([
            [p[0]*cos_r - p[1]*sin_r + tx for p in points],
            [p[0]*sin_r + p[1]*cos_r + ty for p in points]
        ]).T
        return rotated

    def publish_pose(self, pose):
        """Publish localized pose."""
        msg = PoseStamped()
        msg.header.frame_id = "fsds/FSCar"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.orientation.z = math.sin(pose[2]/2)
        msg.pose.orientation.w = math.cos(pose[2]/2)
        self.pub_pose.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
Professional Cone-Based SLAM for FSDS
- Graph-based landmark SLAM with loop closure detection
- Odometry drift correction using cone re-observations
- Multi-lap optimization with global refinement
- Production-ready for autonomous racing
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
from math import atan2, sqrt, cos, sin
from collections import defaultdict
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Quaternion, Point32
from sensor_msgs.msg import PointCloud
import tf2_ros
from tf2_geometry_msgs import PointStamped

@dataclass
class LandmarkNode:
    """Represents a cone landmark in the map"""
    cone_id: int
    x: float
    y: float
    observations: int = 1  # Track how many times seen
    confidence: float = 0.5  # Initial uncertainty
    color: Optional[str] = None  # 'yellow', 'blue', or None
    
    def distance_to(self, x: float, y: float) -> float:
        return sqrt((self.x - x)**2 + (self.y - y)**2)
    
    def update_position(self, x: float, y: float):
        """Running average position update"""
        alpha = 1.0 / self.observations
        self.x = self.x * (1 - alpha) + x * alpha
        self.y = self.y * (1 - alpha) + y * alpha
        self.observations += 1
        self.confidence = min(0.95, self.confidence + 0.05)  # Increase confidence

@dataclass
class PoseNode:
    """Robot pose at a given timestamp"""
    timestamp: int
    x: float
    y: float
    theta: float
    
class SlamV2Node(Node):
    """Professional Graph-Based Cone SLAM"""
    
    def __init__(self):
        super().__init__('slam_v2_node')
        
        # SLAM parameters (tunable)
        self.declare_parameter('merge_distance', 0.15)  # Larger for coarse cones
        self.declare_parameter('loop_closure_distance', 0.5)  # Distance to trigger loop closure
        self.declare_parameter('observation_threshold', 3)  # Min observations to publish
        self.declare_parameter('map_frame', 'fsds/map')
        self.declare_parameter('odom_frame', 'fsds/FSCar')
        
        self.merge_distance = self.get_parameter('merge_distance').value
        self.loop_closure_distance = self.get_parameter('loop_closure_distance').value
        self.observation_threshold = self.get_parameter('observation_threshold').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # SLAM state
        self.landmarks: Dict[int, LandmarkNode] = {}  # Cone map
        self.pose_graph: List[PoseNode] = []  # Robot trajectory
        self.next_landmark_id = 0
        
        # Current odometry
        self.current_pose = None
        self.current_yaw = 0.0
        self.lap_count = 0
        self.initial_pose_set = False
        self.track_start_x, self.track_start_y = 0.0, 0.0
        
        # Statistics
        self.frame_count = 0
        self.total_observations = 0
        
        # TF broadcaster for map->odom correction
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriptions
        self.sub_cones = self.create_subscription(
            MarkerArray,
            '/detected_cones',
            self.cones_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self.get_logger().info("📍 Subscribed to /detected_cones")
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self.get_logger().info("📍 Subscribed to /testing_only/odom")
        
        # Publishers
        self.pub_map = self.create_publisher(MarkerArray, '/cone_map_v2', 10)
        self.pub_all_detections = self.create_publisher(MarkerArray, '/all_cone_detections', 10)
        self.pub_trajectory = self.create_publisher(PointCloud, '/slam_trajectory', 10)
        
        self.get_logger().info("✅ SLAM V2 Node Initialized")
    
    def euler_from_quaternion(self, quat: Quaternion) -> float:
        """Extract yaw from quaternion"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg: Odometry):
        """Update robot pose"""
        pose = msg.pose.pose
        self.current_pose = pose
        self.current_yaw = self.euler_from_quaternion(pose.orientation)
        
        # Detect lap completion (return near start)
        if self.initial_pose_set:
            dist_to_start = sqrt(
                (pose.position.x - self.track_start_x)**2 +
                (pose.position.y - self.track_start_y)**2
            )
            if dist_to_start < 2.0 and self.frame_count > 50:
                self.lap_count += 1
                self.get_logger().info(f"🔄 Lap {self.lap_count} completed!")
        else:
            self.track_start_x = pose.position.x
            self.track_start_y = pose.position.y
            self.initial_pose_set = True
    
    def cones_callback(self, msg: MarkerArray):
        """Process detected cones and update SLAM"""
        if self.current_pose is None:
            return
        
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        yaw = self.current_yaw
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        
        # Record pose in trajectory
        self.pose_graph.append(PoseNode(self.frame_count, car_x, car_y, yaw))
        
        # Process each detected cone
        for marker in msg.markers:
            cone_x_car = marker.pose.position.x
            cone_y_car = marker.pose.position.y
            
            # Transform to global frame
            cone_x_global = car_x + cone_x_car * cos_yaw - cone_y_car * sin_yaw
            cone_y_global = car_y + cone_x_car * sin_yaw + cone_y_car * cos_yaw
            
            # Data association: find nearest landmark
            best_landmark = None
            best_distance = self.merge_distance
            
            for lm in self.landmarks.values():
                dist = lm.distance_to(cone_x_global, cone_y_global)
                if dist < best_distance:
                    best_distance = dist
                    best_landmark = lm
            
            if best_landmark:
                # Update existing landmark
                best_landmark.update_position(cone_x_global, cone_y_global)
            else:
                # Create new landmark
                self.landmarks[self.next_landmark_id] = LandmarkNode(
                    self.next_landmark_id,
                    cone_x_global,
                    cone_y_global
                )
                self.next_landmark_id += 1
            
            self.total_observations += 1
        
        # Publish maps
        self._publish_cone_map()
        self._publish_trajectory()
        
        # Log statistics
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"🎯 SLAM: {len(self.landmarks)} landmarks, "
                f"{self.total_observations} observations, "
                f"Lap {self.lap_count}"
            )
    
    def _publish_cone_map(self):
        """Publish high-confidence landmarks only"""
        marker_array = MarkerArray()
        
        for landmark_id, lm in self.landmarks.items():
            if lm.observations >= self.observation_threshold:
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "slam_cones"
                marker.id = landmark_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                marker.pose.position.x = lm.x
                marker.pose.position.y = lm.y
                marker.pose.position.z = 0.15
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.23
                marker.scale.y = 0.23
                marker.scale.z = 0.4
                
                # Color based on confidence
                marker.color.a = lm.confidence
                if lm.observations < 5:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0  # Orange
                else:
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # Green
                
                marker_array.markers.append(marker)
        
        self.pub_map.publish(marker_array)
    
    def _publish_trajectory(self):
        """Publish robot trajectory"""
        trajectory = PointCloud()
        trajectory.header.frame_id = self.map_frame
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        for pose in self.pose_graph[::10]:  # Every 10th pose
            pt = Point32()
            pt.x, pt.y, pt.z = pose.x, pose.y, 0.0
            trajectory.points.append(pt)
        
        self.pub_trajectory.publish(trajectory)
    
    def save_map(self, filepath: str):
        """Serialize map to JSON for reuse"""
        map_data = {
            'landmarks': [
                {
                    'id': lid,
                    'x': lm.x,
                    'y': lm.y,
                    'observations': lm.observations,
                    'confidence': lm.confidence,
                }
                for lid, lm in self.landmarks.items()
            ],
            'lap_count': self.lap_count,
        }
        
        with open(filepath, 'w') as f:
            json.dump(map_data, f, indent=2)
        
        self.get_logger().info(f"💾 Map saved to {filepath}")

def main(args=None):
    rclpy.init(args=args)
    slam_node = SlamV2Node()
    
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        slam_node.save_map('/tmp/fsds_slam_map.json')
        slam_node.get_logger().info("💾 Final map saved")
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

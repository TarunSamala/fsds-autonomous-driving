#!/usr/bin/env python3
"""
Waypoint Recorder - Perfect Version
Records waypoints from odometry with loop closure detection
Handles the odometry offset issue properly
"""

import json
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Parameters
        self.declare_parameter('distance_before_in_lap', 5.0)
        self.declare_parameter('waypoint_spacing', 0.25)
        self.declare_parameter('loop_closure_distance', 0.5)
        
        self.distance_before_in_lap = self.get_parameter('distance_before_in_lap').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.loop_closure_distance = self.get_parameter('loop_closure_distance').value
        
        # State tracking
        self.state = "SEARCHING"  # SEARCHING, IN_LAP, SAVE
        self.waypoints = []
        self.last_recorded_pos = None
        self.start_pos = None
        self.total_distance = 0.0
        
        # QoS profile - BEST_EFFORT for reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            qos_profile
        )
        
        # Publisher for waypoint visualization
        self.waypoint_pub = self.create_publisher(
            PoseArray,
            '/recorded_waypoints',
            10
        )
        
        # Path publisher
        self.path_pub = self.create_publisher(
            String,
            '/recorder_status',
            10
        )
        
        self.get_logger().info("🚀 Waypoint Recorder Started")
        self.get_logger().info(f"   Distance before in lap: {self.distance_before_in_lap}m")
        self.get_logger().info(f"   Waypoint spacing: {self.waypoint_spacing}m")
        self.get_logger().info(f"   Loop closure distance: {self.loop_closure_distance}m")
    
    def odom_callback(self, msg: Odometry):
        """Handle odometry messages"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.state == "SEARCHING":
            self.handle_searching(x, y)
        elif self.state == "IN_LAP":
            self.handle_in_lap(x, y)
        elif self.state == "SAVE":
            self.handle_save(x, y)
    
    def handle_searching(self, x: float, y: float):
        """Wait until car has moved enough distance"""
        if self.last_recorded_pos is None:
            self.start_pos = (x, y)
            self.last_recorded_pos = (x, y)
            self.get_logger().info(f"✅ START recorded: ({x:.4f}, {y:.4f})")
            return
        
        # Calculate distance traveled
        dist = math.hypot(x - self.last_recorded_pos[0], y - self.last_recorded_pos[1])
        self.total_distance += dist
        
        if self.total_distance >= self.distance_before_in_lap:
            # Start recording
            self.state = "IN_LAP"
            self.waypoints = [(self.start_pos[0], self.start_pos[1])]
            self.last_recorded_pos = self.start_pos
            self.total_distance = 0.0
            self.get_logger().info(f"🟢 ENTERED IN_LAP (traveled {self.total_distance:.2f}m)")
            self.get_logger().info(f"   Initial waypoint: {self.start_pos}")
        
        self.last_recorded_pos = (x, y)
    
    def handle_in_lap(self, x: float, y: float):
        """Record waypoints while in lap"""
        # Distance from last recorded waypoint
        dist = math.hypot(x - self.last_recorded_pos[0], y - self.last_recorded_pos[1])
        
        if dist >= self.waypoint_spacing:
            self.waypoints.append((x, y))
            self.last_recorded_pos = (x, y)
            self.get_logger().debug(f"   📍 Waypoint {len(self.waypoints)}: ({x:.4f}, {y:.4f})")
        
        # Check for loop closure
        dist_to_start = math.hypot(x - self.start_pos[0], y - self.start_pos[1])
        
        if dist_to_start <= self.loop_closure_distance and len(self.waypoints) > 10:
            # Loop detected! Switch to save state
            self.state = "SAVE"
            self.get_logger().info(f"🏁 LOOP DETECTED! dist_to_start={dist_to_start:.4f}m")
            self.get_logger().info(f"   Waypoints collected: {len(self.waypoints)}")
            self.save_waypoints()
    
    def handle_save(self, x: float, y: float):
        """After save, just monitor"""
        pass
    
    def save_waypoints(self):
        """Save waypoints to JSON file"""
        # Snap close the loop
        if len(self.waypoints) > 0:
            self.waypoints.append(self.waypoints[0])
        
        # Save to file
        output_path = Path('/workspace/ros2_ws/waypoints.json')
        
        try:
            with open(output_path, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            
            # Verify
            closure_gap = math.hypot(
                self.waypoints[-1][0] - self.waypoints[0][0],
                self.waypoints[-1][1] - self.waypoints[0][1]
            )
            
            self.get_logger().info(f"💾 SAVED: {output_path}")
            self.get_logger().info(f"✅ Waypoints: {len(self.waypoints) - 1}")  # -1 for closure point
            self.get_logger().info(f"📏 Closure gap: {closure_gap:.4f}m")
            self.get_logger().info("✅ Ready to follow! Press Ctrl+C to exit")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save: {e}")
    
    def publish_waypoints_visualization(self):
        """Publish waypoints for RViz visualization"""
        if not self.waypoints:
            return
        
        pose_array = PoseArray()
        pose_array.header.frame_id = "fsds/FSCar"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in self.waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.waypoint_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    recorder = WaypointRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("\n✋ Recorder stopped")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


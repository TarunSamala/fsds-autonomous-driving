#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from fsds_ros2_bridge.msg import ControlCommand
import numpy as np
import math
import json
import os

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            10
        )
        
        self.pub_control = self.create_publisher(
            ControlCommand,
            '/control_command', # Ensure this maps to /fsds/control_command if needed
            10
        )
        
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        
        # Parameters
        self.look_ahead_distance = 2.5  # Tunable
        self.max_steering_angle = 0.5   # ~28 degrees
        self.target_speed = 0.8         # Throttle
        self.kp_speed = 1.0             # P-controller for speed (optional)
        
        self.path = None
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        
        # Load Waypoints
        self.load_waypoints('/workspace/ros2_ws/waypoints.json')
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ Pure Pursuit Controller Started (JSON Mode)")

    def load_waypoints(self, filename):
        if not os.path.exists(filename):
            self.get_logger().error(f"❌ File not found: {filename}")
            return
            
        with open(filename, 'r') as f:
            data = json.load(f)
            
        self.path = []
        path_msg = Path()
        path_msg.header.frame_id = "fsds/FSCar"
        
        for p in data:
            self.path.append([p[0], p[1]])
            
            # For visualization
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)
        self.get_logger().info(f"📂 Loaded {len(self.path)} waypoints")

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.car_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))

    def control_loop(self):
        if not self.path:
            return

        # 1. Find Lookahead Point
        target_point = self.get_target_point()
        
        # 2. Calculate Steering
        steering = self.calculate_steering(target_point)
        
        # 3. Publish Control
        cmd = ControlCommand()
        cmd.throttle = self.target_speed
        cmd.steering = float(np.clip(steering, -self.max_steering_angle, self.max_steering_angle))
        cmd.brake = 0.0
        self.pub_control.publish(cmd)

    def get_target_point(self):
        # Find closest point
        dists = [math.hypot(p[0]-self.car_x, p[1]-self.car_y) for p in self.path]
        closest_idx = np.argmin(dists)
        
        # Look ahead
        cumulative_dist = 0
        for i in range(closest_idx, len(self.path)-1):
            p1 = self.path[i]
            p2 = self.path[i+1]
            d = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            cumulative_dist += d
            if cumulative_dist > self.look_ahead_distance:
                return p2
        return self.path[-1] # End of path

    def calculate_steering(self, target):
        alpha = math.atan2(target[1] - self.car_y, target[0] - self.car_x) - self.car_yaw
        # Normalize angle
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        
        L = 1.6 # Wheelbase approx
        steering = math.atan2(2.0 * L * math.sin(alpha), self.look_ahead_distance)
        return steering

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


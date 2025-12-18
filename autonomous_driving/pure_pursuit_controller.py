#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
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
            '/control_command',
            10
        )
        
        self.look_ahead_distance = 0.8
        self.max_steering_angle = 0.6
        self.target_speed = 0.02
        
        self.path = []
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        
        self.load_waypoints()
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ Pure Pursuit Controller Started (fs_msgs Mode)")

    def load_waypoints(self):
        filename = '/workspace/ros2_ws/waypoints.json'
        if not os.path.exists(filename):
            self.get_logger().error(f"❌ Waypoints file not found: {filename}")
            return
            
        with open(filename, 'r') as f:
            self.path = json.load(f)
        self.get_logger().info(f"📂 Loaded {len(self.path)} waypoints")

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.car_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))

    def control_loop(self):
        if not self.path:
            return

        target = self.get_target_point()
        steering = self.calculate_steering(target)
        
        cmd = ControlCommand()
        cmd.throttle = float(self.target_speed)
        cmd.steering = float(np.clip(steering, -self.max_steering_angle, self.max_steering_angle))
        cmd.brake = 0.0
        self.pub_control.publish(cmd)

    def get_target_point(self):
        dists = [math.hypot(p[0]-self.car_x, p[1]-self.car_y) for p in self.path]
        closest_idx = np.argmin(dists)
        
        cumulative_dist = 0
        for i in range(closest_idx, len(self.path)-1):
            p1 = self.path[i]
            p2 = self.path[i+1]
            d = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            cumulative_dist += d
            if cumulative_dist >= self.look_ahead_distance:
                return p2
        return self.path[-1]

    def calculate_steering(self, target):
        alpha = math.atan2(target[1] - self.car_y, target[0] - self.car_x) - self.car_yaw
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        
        steering = math.atan2(2.0 * math.sin(alpha), self.look_ahead_distance)
        return steering

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


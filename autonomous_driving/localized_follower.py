#!/usr/bin/env python3
"""
PRODUCTION Follower - FIXED Timer Bug
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
import json
import math
import threading
import time


def euler_from_quaternion(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ProductionFollower(Node):
    def __init__(self):
        super().__init__('production_follower')

        self.speed = 0.04
        self.lookahead_distance = 0.6
        self.kp_steering = 1.2
        
        # Load assets
        self.waypoints = self.load_json('/workspace/ros2_ws/waypoints.json')
        self.cone_map = self.load_json('/workspace/ros2_ws/cone_map.json')
        
        print(f"✅ Loaded {len(self.waypoints)} waypoints")
        print(f"✅ Loaded {len(self.cone_map)} cones")
        
        # State
        self.odom_pose = None
        self.odom_yaw = 0.0
        self.debug_counter = 0
        
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.kickstart_done = False

        # Publishers/Subscriptions
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10
        )

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.kickstart()

    def load_json(self, filename):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            return [[float(p[0]), float(p[1])] for p in data]
        except:
            self.get_logger().warn(f"No {filename}")
            return []

    def kickstart(self):
        def kick():
            print("[KICKSTART] 0.25 throttle...")
            for _ in range(3):
                msg = ControlCommand()
                msg.throttle = 0.25; msg.steering = 0.0; msg.brake = 0.0
                self.cmd_pub.publish(msg)
                time.sleep(0.1)
            
            print("[KICKSTART] Brake...")
            for _ in range(8):
                msg = ControlCommand()
                msg.throttle = 0.0; msg.steering = 0.0; msg.brake = 0.4
                self.cmd_pub.publish(msg)
                time.sleep(0.1)
                
            self.kickstart_done = True
            print("✅ PRODUCTION RACING ACTIVE! 🏎️🔄")
            
        threading.Thread(target=kick, daemon=True).start()

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def find_closest_waypoint(self):
        if not self.waypoints or self.odom_pose is None:
            return 0, float('inf')
            
        cx, cy = self.odom_pose.position.x, self.odom_pose.position.y
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (wx, wy) in enumerate(self.waypoints):
            dist = math.hypot(wx - cx, wy - cy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        return closest_idx, min_dist

    def find_lookahead(self, closest_idx):
        cx, cy = self.odom_pose.position.x, self.odom_pose.position.y
        
        for i in range(closest_idx, len(self.waypoints)):
            idx = i % len(self.waypoints)
            wx, wy = self.waypoints[idx]
            dist = math.hypot(wx - cx, wy - cy)
            if dist > self.lookahead_distance:
                return wx, wy, idx
                
        return self.waypoints[closest_idx][0], self.waypoints[closest_idx][1], closest_idx

    def control_loop(self):
        if not self.kickstart_done or self.odom_pose is None:
            return

        # Find target
        closest_idx, dist_to_track = self.find_closest_waypoint()
        tx, ty, target_idx = self.find_lookahead(closest_idx)
        
        # Pure pursuit steering
        dx = tx - self.odom_pose.position.x
        dy = ty - self.odom_pose.position.y
        target_angle = math.atan2(dy, dx)
        
        angle_error = target_angle - self.odom_yaw
        angle_error = (angle_error + math.pi) % (2*math.pi) - math.pi
        
        self.steering = self.kp_steering * math.sin(angle_error)
        self.steering = max(-1.0, min(1.0, self.steering))
        
        # Speed control
        speed_factor = max(0.3, 1.0 - dist_to_track * 0.5)
        self.throttle = self.speed * speed_factor
        self.brake = 0.0
        
        # Publish
        msg = ControlCommand()
        msg.throttle = float(self.throttle)
        msg.steering = float(self.steering)
        msg.brake = float(self.brake)
        self.cmd_pub.publish(msg)
        
        # FIXED Debug counter
        self.debug_counter += 1
        if self.debug_counter % 100 == 0:
            print(f"Track:{dist_to_track:.2f}m → Target:{target_idx} steer:{self.steering:.2f} speed:{self.throttle:.3f}")

def main():
    rclpy.init()
    node = ProductionFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


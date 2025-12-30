#!/usr/bin/env python3
"""
LOCALIZED Pure Pursuit - Uses Cone Map for Position Correction
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from fs_msgs.msg import ControlCommand
import numpy as np
import math
import json
import threading
import time


def euler_from_quaternion(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalizedPurePursuit(Node):
    def __init__(self):
        super().__init__('localized_follower')

        # Parameters
        self.speed = 0.035
        self.lookahead_distance = 0.6
        self.cone_tolerance = 1.5  # Use cones within 1.5m for localization
        
        # State
        self.waypoints = []
        self.cone_map = {}  # From SLAM: (x,y) → cone_data
        self.odom_pose = None
        self.odom_yaw = 0.0
        self.localized_pose = None  # Corrected pose
        
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.kickstart_done = False

        # Load waypoints
        self.load_waypoints()
        
        # Publishers/Subscriptions
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10
        )
        self.cones_sub = self.create_subscription(
            MarkerArray, '/cone_map', self.cones_callback, 10
        )

        # Timers
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        print("=" * 80)
        print("🗺️  LOCALIZED PURE PURSUIT - CONE CORRECTION")
        print("=" * 80)
        print(f"✅ Loaded {len(self.waypoints)} waypoints")
        print("🔍 Using /cone_map for localization correction")
        print("🚀 KICKSTART...")
        
        self.kickstart()

    def kickstart(self):
        """Quick kickstart then autonomous."""
        def kick():
            # Throttle burst
            for _ in range(3):
                msg = ControlCommand()
                msg.throttle = 0.25; msg.steering = 0.0; msg.brake = 0.0
                self.cmd_pub.publish(msg)
                time.sleep(0.1)
            
            # Brake
            for _ in range(8):
                msg = ControlCommand()
                msg.throttle = 0.0; msg.steering = 0.0; msg.brake = 0.4
                self.cmd_pub.publish(msg)
                time.sleep(0.1)
                
            self.kickstart_done = True
            print("✅ LOCALIZED CONTROL ACTIVE!")
            
        thread = threading.Thread(target=kick, daemon=True)
        thread.start()

    def load_waypoints(self):
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                data = json.load(f)
            self.waypoints = np.array([[float(wp[0]), float(wp[1])] for wp in data])
            print(f"📍 {len(self.waypoints)} waypoints loaded")
        except:
            print("⚠️ No waypoints - will use cone centerline")

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def cones_callback(self, msg):
        """Update cone map from SLAM."""
        self.cone_map.clear()
        for marker in msg.markers:
            if marker.ns == "global_cone_map":
                self.cone_map[(marker.pose.position.x, marker.pose.position.y)] = True

    def localize_with_cones(self):
        """Correct odometry using nearby cones."""
        if self.odom_pose is None or len(self.cone_map) < 10:
            self.localized_pose = self.odom_pose
            return
            
        ox, oy = self.odom_pose.position.x, self.odom_pose.position.y
        
        # Find nearby cones
        nearby_cones = []
        for cx, cy in self.cone_map.keys():
            dist = math.hypot(cx - ox, cy - oy)
            if dist < self.cone_tolerance:
                nearby_cones.append([cx, cy])
                
        if len(nearby_cones) < 3:
            self.localized_pose = self.odom_pose
            return
            
        # Estimate lateral offset using cone distribution
        cones = np.array(nearby_cones)
        cone_center_x = np.mean(cones[:, 0])
        cone_center_y = np.mean(cones[:, 1])
        
        # Correct lateral position toward cone centerline
        lateral_error = oy - cone_center_y
        corrected_y = oy - 0.3 * lateral_error  # 30% correction gain
        
        # Update localized pose
        self.localized_pose = self.odom_pose
        self.localized_pose.position.y = corrected_y

    def find_lookahead_point(self):
        """Find lookahead waypoint using localized pose."""
        if self.localized_pose is None or len(self.waypoints) == 0:
            return 0.0, 0.0
            
        lx, ly = self.localized_pose.position.x, self.localized_pose.position.y
        
        closest_idx = 0
        min_dist = float('inf')
        
        # Find closest waypoint
        for i, (wx, wy) in enumerate(self.waypoints):
            dist = math.hypot(wx - lx, wy - ly)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        # Find lookahead point
        for i in range(closest_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i % len(self.waypoints)]
            dist = math.hypot(wx - lx, wy - ly)
            if dist > self.lookahead_distance:
                return wx, wy
                
        # Fallback
        return self.waypoints[closest_idx]

    def control_loop(self):
        if not self.kickstart_done or self.localized_pose is None:
            return
            
        # 1. Localize using cones
        self.localize_with_cones()
        
        # 2. Find lookahead
        lx, ly = self.find_lookahead_point()
        
        # 3. Pure pursuit steering
        dx = lx - self.localized_pose.position.x
        dy = ly - self.localized_pose.position.y
        
        angle_error = math.atan2(dy, dx) - self.odom_yaw
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        
        self.steering = math.tan(angle_error) * self.lookahead_distance
        self.steering = max(-1.0, min(1.0, self.steering))
        
        # 4. Speed control
        self.throttle = self.speed
        self.brake = 0.0
        
        # Publish
        msg = ControlCommand()
        msg.throttle = float(self.throttle)
        msg.steering = float(self.steering)
        msg.brake = float(self.brake)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = LocalizedPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


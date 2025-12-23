#!/usr/bin/env python3
"""
waypoint_follower.py - FSDS Optimized
Handles spawn mismatch + wrong orientation + visualization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from fs_msgs.msg import ControlCommand
import json
import math
import numpy as np
import threading
import time


def euler_from_quaternion(quat):
    """Extract yaw from quaternion (FSDS optimized)"""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw  # No flip - FSDS convention


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Tunable parameters
        self.declare_parameter('lookahead', 1.2)  # Increased for stability
        self.declare_parameter('target_speed', 0.02)
        self.declare_parameter('max_steer', 0.8)  # Reduced for safety
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('goal_tolerance', 1.0)  # Increased tolerance
        
        self.lookahead = self.get_parameter('lookahead').value
        self.target_speed = self.get_parameter('target_speed').value
        self.max_steer = self.get_parameter('max_steer').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # State
        self.waypoints = []
        self.closest_idx = 0
        self.current_pose = None
        self.current_yaw = 0.0
        self.kickstart_done = False
        
        # Publishers
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_pub = self.create_publisher(Path, '/follow_path', 10)
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.vis_timer = self.create_timer(0.2, self.publish_viz)
        
        self.load_waypoints()
        
        self.get_logger().info('🤖 WAYPOINT FOLLOWER (SPAWN-AWARE)')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        if len(self.waypoints) > 0:
            self.get_logger().info(f'WP[0]: {self.waypoints[0]}')
        
        # Kickstart
        self.kick_thread = threading.Thread(target=self.kickstart, daemon=True)
        self.kick_thread.start()
    
    def kickstart(self):
        """Kickstart sequence"""
        self.get_logger().info('🚀 KICKSTART...')
        for _ in range(5):
            msg = ControlCommand()
            msg.throttle = 0.2
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        
        for _ in range(10):
            msg = ControlCommand()
            msg.brake = 0.5
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        
        self.kickstart_done = True
        self.get_logger().info('✅ READY FOR AUTONOMOUS')
    
    def load_waypoints(self):
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                data = json.load(f)
                self.waypoints = [[float(x), float(y)] for x, y in data]
        except Exception as e:
            self.get_logger().error(f'Load failed: {e}')
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)
    
    def find_closest_waypoint(self):
        """Find closest waypoint (forward search window)"""
        if self.current_pose is None or not self.waypoints:
            return 0
        
        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        min_dist = float('inf')
        closest_idx = 0
        
        # Search forward from current index (±10 waypoints)
        start_idx = max(0, self.closest_idx - 10)
        end_idx = min(len(self.waypoints), self.closest_idx + 20)
        
        for i in range(start_idx, end_idx):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.closest_idx = closest_idx
        return closest_idx
    
    def find_lookahead_target(self):
        """Find lookahead point ahead of closest waypoint"""
        closest_idx = self.find_closest_waypoint()
        
        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        
        # Look ahead from closest waypoint
        for i in range(closest_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist >= self.lookahead:
                return i, (wx, wy)
        
        # Loop back to start if at end
        for i in range(0, closest_idx):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist >= self.lookahead:
                return i, (wx, wy)
        
        return len(self.waypoints) - 1, self.waypoints[-1]
    
    def pure_pursuit(self, target_x, target_y):
        """Pure pursuit steering"""
        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        
        dx = target_x - cx
        dy = target_y - cy
        L = math.hypot(dx, dy)
        
        if L < 0.01:
            return 0.0
        
        alpha = math.atan2(dy, dx) - self.current_yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi  # Normalize [-pi, pi]
        
        # Pure pursuit: steering = atan(2 * V * sin(alpha) / L)
        # Simplified for constant speed:
        steering = 2 * math.sin(alpha) / L * self.wheelbase
        return np.clip(steering, -self.max_steer, self.max_steer)
    
    def control_loop(self):
        if not self.kickstart_done or self.current_pose is None:
            return
        
        target_idx, (tx, ty) = self.find_lookahead_target()
        steering = self.pure_pursuit(tx, ty)
        
        # Speed scaling for safety
        speed_scale = max(0.3, 1.0 - 0.7 * abs(steering))
        throttle = self.target_speed * speed_scale
        
        # Goal check
        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        goal_dist = math.hypot(self.waypoints[-1][0] - cx, self.waypoints[-1][1] - cy)
        if goal_dist < self.goal_tolerance:
            self.get_logger().info('✅ GOAL REACHED!')
            throttle = 0.0
        
        # Publish
        cmd = ControlCommand()
        cmd.throttle = float(throttle)
        cmd.steering = float(steering)
        cmd.brake = 0.0
        self.cmd_pub.publish(cmd)
    
    def publish_viz(self):
        """RViz visualization"""
        if not self.waypoints:
            return
        
        # Waypoint markers
        markers = MarkerArray()
        cx, cy = (self.current_pose.position.x if self.current_pose else 0,
                  self.current_pose.position.y if self.current_pose else 0)
        
        for i, (wx, wy) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'fsds/FSCar'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.SPHERE
            m.pose.position.x, m.pose.position.y = wx, wy
            m.scale.x = m.scale.y = m.scale.z = 0.15
            
            # Color coding
            dist = math.hypot(wx - cx, wy - cy)
            if i == self.closest_idx:
                m.color.r = 1.0; m.color.g = 0.0  # Red = closest
            elif dist < self.lookahead:
                m.color.r = 1.0; m.color.g = 1.0  # Yellow = lookahead zone
            else:
                m.color.g = 1.0  # Green = far waypoints
            m.color.a = 0.8
            markers.markers.append(m)
        
        self.markers_pub.publish(markers)
        
        # Path
        path = Path()
        path.header.frame_id = 'fsds/FSCar'
        path.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.waypoints:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = wx, wy
            path.poses.append(p)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

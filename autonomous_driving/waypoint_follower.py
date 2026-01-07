#!/usr/bin/env python3
"""
Pure Pursuit Waypoint Follower - CROSS-TRACK ERROR AWARE
Includes lateral error correction and reduced speed for tight tracks
"""

import rclpy
import json
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand

def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.declare_parameter('throttle', 0.03)  # REDUCED from 0.05
        self.declare_parameter('lookahead', 0.8)   # REDUCED from 1.8
        self.declare_parameter('wheelbase', 0.4)
        self.declare_parameter('max_cross_track_error', 0.5)  # NEW: max allowed lateral drift
        
        self.throttle = self.get_parameter('throttle').value
        self.lookahead = self.get_parameter('lookahead').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_cte = self.get_parameter('max_cross_track_error').value
        
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded - exiting")
            return
        
        self.pos = None
        self.yaw = 0.0
        self.current_idx = 0
        self.lap_count = 0
        self.last_lap_idx = -100
        
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback,
            rclpy.qos.QoSProfile(depth=1)
        )
        
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("🚗 PURE PURSUIT - CROSS-TRACK ERROR AWARE")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info(f"📊 Throttle: {self.throttle}, Lookahead: {self.lookahead}m, Wheelbase: {self.wheelbase}m")
        self.get_logger().info(f"⚠️  Max cross-track error: {self.max_cte}m")
        self.get_logger().info("=" * 80)

    def load_waypoints(self):
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                waypoints = json.load(f)
            self.get_logger().info(f"Waypoints loaded: {len(waypoints)} points")
            return waypoints
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
            return []

    def odom_callback(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

    def find_closest_waypoint(self):
        """Find absolute closest waypoint"""
        if not self.waypoints or self.pos is None:
            return 0
        
        min_dist = float('inf')
        closest_idx = 0
        
        for i, wp in enumerate(self.waypoints):
            dist = math.hypot(wp[0] - self.pos[0], wp[1] - self.pos[1])
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx

    def compute_cross_track_error(self, closest_idx):
        """Compute lateral distance from nearest waypoint"""
        if not self.waypoints or self.pos is None:
            return 0.0
        
        wp = self.waypoints[closest_idx]
        return math.hypot(wp[0] - self.pos[0], wp[1] - self.pos[1])

    def find_lookahead_point(self):
        """Find target point at lookahead distance ahead"""
        if not self.waypoints or self.pos is None:
            return self.waypoints[0] if self.waypoints else (0, 0)
        
        for i in range(len(self.waypoints)):
            idx = (self.current_idx + i) % len(self.waypoints)
            wp = self.waypoints[idx]
            dist = math.hypot(wp[0] - self.pos[0], wp[1] - self.pos[1])
            if dist > self.lookahead:
                return wp
        
        return self.waypoints[(self.current_idx + 1) % len(self.waypoints)]

    def compute_steering(self, target):
        """Pure Pursuit steering law"""
        if self.pos is None:
            return 0.0
        
        dx = target[0] - self.pos[0]
        dy = target[1] - self.pos[1]
        distance = math.hypot(dx, dy)
        
        if distance < 0.01:
            return 0.0
        
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.yaw)
        
        steering = math.atan2(2 * self.wheelbase * math.sin(heading_error), distance)
        steering = max(-1.0, min(1.0, steering))
        
        return float(steering)

    def control_loop(self):
        if self.pos is None or not self.waypoints:
            return
        
        self.current_idx = self.find_closest_waypoint()
        cte = self.compute_cross_track_error(self.current_idx)
        
        # Check for excessive drift
        if cte > self.max_cte:
            self.get_logger().warn(f"⚠️  DRIFT: CTE={cte:.2f}m > max={self.max_cte}m at WP{self.current_idx}")
        
        target = self.find_lookahead_point()
        steering = self.compute_steering(target)
        
        msg = ControlCommand()
        msg.throttle = self.throttle
        msg.steering = steering
        msg.brake = 0.0
        self.cmd_pub.publish(msg)
        
        # Log every 10 iterations (~0.5s)
        if self.current_idx % 10 == 0:
            dist_to_target = math.hypot(target[0] - self.pos[0], target[1] - self.pos[1])
            self.get_logger().info(
                f"🚗 Pos({self.pos[0]:6.2f}, {self.pos[1]:6.2f}) "
                f"WP{self.current_idx:3d} "
                f"CTE{cte:.2f}m "
                f"Steer{steering:6.3f} "
                f"Target_Dist{dist_to_target:5.2f}m"
            )

def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuitFollower()
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.get_logger().info("Interrupted by user")
    finally:
        follower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()


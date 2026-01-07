#!/usr/bin/env python3
"""
Waypoint Follower - Enhanced Pure Pursuit with Real-time Diagnostics
Follows recorded waypoints with CTE monitoring and status indicators
"""

import json
import math
from pathlib import Path
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String


class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Parameters
        self.declare_parameter('throttle', 0.025)
        self.declare_parameter('lookahead', 0.6)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('steering_rate_limit', 0.3)
        
        self.throttle = self.get_parameter('throttle').value
        self.lookahead = self.get_parameter('lookahead').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_rate_limit = self.get_parameter('steering_rate_limit').value
        
        # State
        self.waypoints = []
        self.current_pos = None
        self.current_heading = 0.0
        self.last_steering = 0.0
        
        # Diagnostics
        self.cte_history = deque(maxlen=100)  # Last 100 cycles
        self.steering_history = deque(maxlen=100)
        self.cycle_count = 0
        self.last_diagnostic_time = self.get_clock().now()
        
        # Load waypoints
        self.load_waypoints()
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            qos_profile
        )
        
        # Publishers
        self.control_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10
        )
        
        self.target_pub = self.create_publisher(
            PoseArray,
            '/waypoint_markers',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/follower_status',
            10
        )
        
        self.get_logger().info("🚗 Pure Pursuit Follower Started")
        self.get_logger().info(f"   Throttle: {self.throttle}")
        self.get_logger().info(f"   Lookahead: {self.lookahead}m")
        self.get_logger().info(f"   Max Steering: {self.max_steering_angle:.2f} rad")
        self.get_logger().info(f"   Steering Rate Limit: {self.steering_rate_limit:.2f} rad/s")
    
    def load_waypoints(self):
        """Load waypoints from JSON file"""
        waypoint_file = Path('/workspace/ros2_ws/waypoints.json')
        
        if not waypoint_file.exists():
            self.get_logger().error(f"❌ Waypoint file not found: {waypoint_file}")
            self.get_logger().error("   Run waypoint_recorder_perfect first!")
            return
        
        try:
            with open(waypoint_file, 'r') as f:
                self.waypoints = json.load(f)
            
            self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints")
            
            # Validate waypoints
            if len(self.waypoints) < 5:
                self.get_logger().error("❌ Not enough waypoints recorded")
                return
            
            # Check closure
            closure = math.hypot(
                self.waypoints[-1][0] - self.waypoints[0][0],
                self.waypoints[-1][1] - self.waypoints[0][1]
            )
            self.get_logger().info(f"📏 Closure gap: {closure:.4f}m")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load waypoints: {e}")
    
    def odom_callback(self, msg: Odometry):
        """Process odometry and control the vehicle"""
        if not self.waypoints:
            return
        
        # Extract position and heading
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract heading from quaternion
        q = msg.pose.pose.orientation
        self.current_heading = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        # Compute Pure Pursuit steering
        steering = self.compute_pure_pursuit()
        
        # Rate limit steering (smooth control)
        steering = self.rate_limit_steering(steering)
        
        # Track for diagnostics
        self.cycle_count += 1
        cte = self.compute_cte()
        self.cte_history.append(cte)
        self.steering_history.append(steering)
        
        # Publish control command
        self.publish_control(steering)
        
        # Log per-cycle info
        current_wp = self.find_nearest_waypoint()
        self.get_logger().debug(
            f"🚗 Pos({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}) → "
            f"WP {current_wp:2d} CTE{cte:7.3f}m Steer {steering:7.4f}"
        )
        
        # Publish diagnostics every 5 seconds (≈250 cycles at 50Hz)
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds > 5e9:
            self.publish_diagnostics()
            self.last_diagnostic_time = now
    
    def compute_pure_pursuit(self) -> float:
        """Compute steering angle using Pure Pursuit algorithm"""
        if not self.waypoints or not self.current_pos:
            return 0.0
        
        # Find target waypoint (lookahead distance)
        target_idx = self.find_lookahead_waypoint()
        target = self.waypoints[target_idx]
        
        # Vector from current pos to target
        dx = target[0] - self.current_pos[0]
        dy = target[1] - self.current_pos[1]
        
        # Distance to target
        distance = math.hypot(dx, dy)
        if distance < 0.01:
            return 0.0
        
        # Angle to target (in world frame)
        angle_to_target = math.atan2(dy, dx)
        
        # Cross-track error (perpendicular distance)
        # Signed CTE: positive = target is left, negative = target is right
        cte = -math.sin(self.current_heading - angle_to_target) * distance
        
        # Pure Pursuit steering law: steering = atan2(2 * L * sin(cross_track_error), v * lookahead)
        # Simplified version with lookahead distance
        steering = math.atan2(2.0 * cte, self.lookahead)
        
        # Clamp to max steering angle
        steering = max(-self.max_steering_angle, min(self.max_steering_angle, steering))
        
        return steering
    
    def find_lookahead_waypoint(self) -> int:
        """Find the waypoint at lookahead distance ahead"""
        if not self.current_pos:
            return 0
        
        current_wp = self.find_nearest_waypoint()
        
        # Search forward for waypoint at lookahead distance
        for i in range(current_wp, len(self.waypoints)):
            dist = math.hypot(
                self.waypoints[i][0] - self.current_pos[0],
                self.waypoints[i][1] - self.current_pos[1]
            )
            if dist >= self.lookahead:
                return i
        
        # If no waypoint found at lookahead distance, use last
        return len(self.waypoints) - 1
    
    def find_nearest_waypoint(self) -> int:
        """Find the nearest waypoint to current position"""
        if not self.waypoints or not self.current_pos:
            return 0
        
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, wp in enumerate(self.waypoints):
            dist = math.hypot(wp[0] - self.current_pos[0], wp[1] - self.current_pos[1])
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def compute_cte(self) -> float:
        """Compute cross-track error"""
        if not self.waypoints or not self.current_pos:
            return 0.0
        
        current_wp = self.find_nearest_waypoint()
        
        # Use current and next waypoint to define the line
        if current_wp >= len(self.waypoints) - 1:
            next_wp = 0  # Loop back
        else:
            next_wp = current_wp + 1
        
        wp1 = self.waypoints[current_wp]
        wp2 = self.waypoints[next_wp]
        
        # Distance from point to line segment
        # Using formula: |ax + by + c| / sqrt(a^2 + b^2)
        x0, y0 = self.current_pos
        x1, y1 = wp1
        x2, y2 = wp2
        
        # Line equation: (y2-y1)x - (x2-x1)y + (x2-x1)y1 - (y2-y1)x1 = 0
        a = y2 - y1
        b = -(x2 - x1)
        c = (x2 - x1) * y1 - (y2 - y1) * x1
        
        numerator = abs(a * x0 + b * y0 + c)
        denominator = math.sqrt(a**2 + b**2)
        
        if denominator < 0.001:
            return 0.0
        
        return numerator / denominator
    
    def rate_limit_steering(self, target_steering: float) -> float:
        """Rate limit steering to smooth control"""
        max_change = self.steering_rate_limit * 0.02  # 0.02s cycle time (50Hz)
        
        delta = target_steering - self.last_steering
        delta = max(-max_change, min(max_change, delta))
        
        result = self.last_steering + delta
        self.last_steering = result
        
        return result
    
    def publish_control(self, steering: float):
        """Publish control command"""
        cmd = ControlCommand()
        cmd.steering = steering
        cmd.throttle = self.throttle
        
        self.control_pub.publish(cmd)
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        if not self.cte_history:
            return
        
        cte_values = list(self.cte_history)
        avg_cte = sum(cte_values) / len(cte_values)
        max_cte = max(cte_values)
        min_cte = min(cte_values)
        
        steering_values = list(self.steering_history)
        avg_steering = sum(steering_values) / len(steering_values)
        max_steering = max(abs(s) for s in steering_values)
        
        # Determine status
        if max_cte < 0.3:
            status = "🟢 GREEN"
        elif max_cte < 0.5:
            status = "🟡 YELLOW"
        else:
            status = "🔴 RED"
        
        msg = (
            f"{status} [DIAG] CTE: avg={avg_cte:.3f}m max={max_cte:.3f}m min={min_cte:.3f}m | "
            f"Steer: avg={avg_steering:+.4f} max={max_steering:.4f} | "
            f"WP#{self.find_nearest_waypoint()}/{len(self.waypoints)} | "
            f"Cycles={self.cycle_count}"
        )
        
        self.get_logger().info(msg)
        
        status_msg = String()
        status_msg.data = msg
        self.status_pub.publish(status_msg)
    
    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion to yaw angle"""
        # Using the formula from quaternion
        sin_roll_cos_pitch = 2 * (w * x + y * z)
        cos_roll_cos_pitch = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)
        
        sin_pitch = 2 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)
        
        sin_yaw_cos_pitch = 2 * (w * z + x * y)
        cos_yaw_cos_pitch = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
        
        return yaw


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuitFollower()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        print("\n✋ Follower stopped")
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3

"""
Pure Pursuit Waypoint Follower - Production Ready

- Loads waypoints from JSON
- Implements Pure Pursuit steering control
- Tunable parameters: throttle, lookahead, wheelbase
- Complete error handling and logging
- Ready for autonomous racing
"""

import rclpy
import json
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import Quaternion


def quat_to_yaw(q: Quaternion) -> float:
    """Convert quaternion to yaw angle (radians)"""
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y**2 + q.z**2)
    )


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class PurePursuitFollower(Node):
    """Pure Pursuit steering controller for waypoint following"""

    def __init__(self):
        super().__init__('waypoint_follower')

        # Declare and load parameters
        self.declare_parameter('throttle', 0.05)
        self.declare_parameter('lookahead', 1.8)
        self.declare_parameter('wheelbase', 0.4)

        self.throttle = self.get_parameter('throttle').value
        self.lookahead = self.get_parameter('lookahead').value
        self.wheelbase = self.get_parameter('wheelbase').value

        # Load waypoints
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            self.get_logger().error("❌ No waypoints loaded - exiting")
            return

        # State
        self.pos = None
        self.yaw = 0.0
        self.current_idx = 0
        self.lap_count = 0
        self.last_lap_idx = -100  # Large distance for first lap detection

        # Publishers
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )

        # Control loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Logging
        self.get_logger().info("=" * 80)
        self.get_logger().info("🚗 PURE PURSUIT WAYPOINT FOLLOWER")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info(f"📊 Parameters:")
        self.get_logger().info(f"   Throttle:   {self.throttle}")
        self.get_logger().info(f"   Lookahead:  {self.lookahead}m")
        self.get_logger().info(f"   Wheelbase:  {self.wheelbase}m")
        self.get_logger().info("=" * 80)

    def load_waypoints(self) -> list:
        """Load waypoints from JSON file"""
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                waypoints = json.load(f)
            self.get_logger().info(f"📁 Waypoints loaded: {len(waypoints)} points")
            return waypoints
        except FileNotFoundError:
            self.get_logger().error("❌ waypoints.json not found!")
            return []
        except json.JSONDecodeError:
            self.get_logger().error("❌ Invalid waypoints.json format!")
            return []
        except Exception as e:
            self.get_logger().error(f"❌ Error loading waypoints: {e}")
            return []

    def odom_callback(self, msg: Odometry):
        """Update pose from odometry"""
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

    def find_closest_waypoint(self) -> int:
        """Find closest waypoint to current position"""
        if not self.waypoints or self.pos is None:
            return 0

        min_dist = float('inf')
        closest_idx = self.current_idx

        for i, wp in enumerate(self.waypoints):
            dist = math.hypot(wp[0] - self.pos[0], wp[1] - self.pos[1])
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def find_lookahead_point(self) -> tuple:
        """Find target point at lookahead distance"""
        if not self.waypoints or self.pos is None:
            return self.waypoints[0] if self.waypoints else (0, 0)

        # Search forward from current position
        for i in range(len(self.waypoints)):
            idx = (self.current_idx + i) % len(self.waypoints)
            wp = self.waypoints[idx]
            dist = math.hypot(wp[0] - self.pos[0], wp[1] - self.pos[1])

            if dist >= self.lookahead:
                return wp

        # Fallback: return next waypoint
        return self.waypoints[(self.current_idx + 1) % len(self.waypoints)]

    def compute_steering(self, target: tuple) -> float:
        """Compute steering angle using Pure Pursuit algorithm"""
        if self.pos is None:
            return 0.0

        # Vector to target
        dx = target[0] - self.pos[0]
        dy = target[1] - self.pos[1]
        distance = math.hypot(dx, dy)

        # Avoid division by zero
        if distance < 0.01:
            return 0.0

        # Target heading
        target_heading = math.atan2(dy, dx)

        # Heading error (normalized)
        heading_error = normalize_angle(target_heading - self.yaw)

        # Pure Pursuit steering law: steering = atan2(2*L*sin(α), d)
        # where α = heading_error, d = distance, L = wheelbase
        steering = math.atan2(
            2 * self.wheelbase * math.sin(heading_error),
            distance
        )

        # Clamp steering to [-1, 1]
        steering = max(-1.0, min(1.0, steering))

        return float(steering)

    def detect_lap_completion(self) -> bool:
        """Detect when car completes a lap"""
        if self.current_idx < self.last_lap_idx + 50:
            return False  # Haven't traveled enough since last lap

        if self.current_idx > len(self.waypoints) * 0.9:  # Near end of waypoints
            if self.current_idx < self.last_lap_idx + len(self.waypoints) * 0.9:
                # Just wrapped around
                return False

            # Lap completed
            self.lap_count += 1
            self.last_lap_idx = self.current_idx
            self.get_logger().info(f"🏁 LAP {self.lap_count} COMPLETED!")
            return True

        return False

    def control_loop(self):
        """Main control loop (20 Hz)"""
        if self.pos is None or not self.waypoints:
            return

        # Find closest waypoint
        self.current_idx = self.find_closest_waypoint()

        # Find lookahead target
        target = self.find_lookahead_point()

        # Compute steering
        steering = self.compute_steering(target)

        # Detect lap completion
        self.detect_lap_completion()

        # Create and publish control command
        msg = ControlCommand()
        msg.throttle = self.throttle
        msg.steering = steering
        msg.brake = 0.0

        self.cmd_pub.publish(msg)

        # Debug logging (every 100 iterations = every 5 seconds)
        if self.current_idx % 10 == 0:
            dist_to_target = math.hypot(
                target[0] - self.pos[0],
                target[1] - self.pos[1]
            )
            self.get_logger().debug(
                f"📍 WP:{self.current_idx:3d} | "
                f"Steering:{steering:6.3f} | "
                f"Dist:{dist_to_target:5.2f}m"
            )

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info(f"🛑 Waypoint follower stopped (Laps: {self.lap_count})")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuitFollower()

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.get_logger().info("⚠️ Interrupted by user")
    finally:
        follower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
Waypoint Follower for FSDS with Proper Kickstart + Angle-Based Steering

ControlCommand scale (from FSDS docs):
  - throttle: 0.0 to 1.0
  - steering: -1.0 (left) to +1.0 (right) [maps to ±25°]
  - brake: 0.0 to 1.0

Uses recorded waypoints with aggressive angle-based steering.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from fs_msgs.msg import ControlCommand
import json
import math
import threading
import time


def euler_from_quaternion(quat):
    """Extract yaw from quaternion."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Tuned parameters
        self.declare_parameter('lookahead_distance', 0.8)    # meters
        self.declare_parameter('target_speed', 0.08)         # throttle (0–1)
        self.declare_parameter('steering_multiplier', 3.5)   # aggressive: angle_rad * 3.5
        self.declare_parameter('max_steering', 1.0)          # clamp to [-1, +1]
        self.declare_parameter('goal_tolerance', 0.7)        # meters

        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.target_speed = self.get_parameter('target_speed').value
        self.steering_multiplier = self.get_parameter('steering_multiplier').value
        self.max_steering = self.get_parameter('max_steering').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # State
        self.waypoints = []
        self.current_idx = 0
        self.current_pose = None
        self.current_yaw = 0.0
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.kickstart_done = False

        # Kickstart params (IDENTICAL to keyboard_control.py)
        self.kickstart_value = 0.2      # throttle during kickstart
        self.kickstart_duration = 0.5   # seconds (0.5s = 5 * 0.1s ticks)
        self.brake_duration = 1.0       # seconds after kickstart

        # Publishers
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_pub = self.create_publisher(Path, '/follow_path', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10
        )

        # Timers
        self.control_timer = self.create_timer(0.1, self.publish_cmd)      # 10 Hz
        self.follow_timer = self.create_timer(0.05, self.follow_waypoints) # 20 Hz
        self.vis_timer = self.create_timer(0.5, self.publish_visualization)

        self.load_waypoints()

        self.get_logger().info('=' * 70)
        self.get_logger().info('🤖 WAYPOINT FOLLOWER (FSDS STEERING SCALE, AGGRESSIVE ANGLE-BASED)')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'✅ Loaded {len(self.waypoints)} waypoints')
        if self.waypoints:
            self.get_logger().info(f'   First WP: [{self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f}]')
        self.get_logger().info(f'   Lookahead: {self.lookahead_distance} m')
        self.get_logger().info(f'   Target speed: {self.target_speed} (throttle 0–1)')
        self.get_logger().info(f'   Steering multiplier: {self.steering_multiplier} (rad → [-1, +1])')
        self.get_logger().info('')
        self.get_logger().info('🚀 KICKSTART INITIATED!')
        self.get_logger().info(f'   Phase 1: throttle={self.kickstart_value} for {self.kickstart_duration}s')
        self.get_logger().info(f'   Phase 2: brake=0.5 for {self.brake_duration}s')
        self.get_logger().info(f'   Then: autonomous waypoint following starts')
        self.get_logger().info('=' * 70)

        # Start kickstart thread
        self.kick_thread = threading.Thread(target=self.kickstart_sequence, daemon=True)
        self.kick_thread.start()

    # ================================================================== #
    # KICKSTART (copied from keyboard_control.py, verified working)
    # ================================================================== #

    def kickstart_sequence(self):
        """
        Identical kickstart to keyboard_control.py:
        1. Send throttle=0.2 for 0.5s (get car moving, overcome friction)
        2. Send brake=0.5 for 1.0s (let it coast, then stop)
        3. Reset all commands, set kickstart_done=True
        """
        # Phase 1: Throttle pulse
        num_throttle_ticks = int(self.kickstart_duration / 0.1)
        self.get_logger().info(f'[KICKSTART] Phase 1: sending {num_throttle_ticks} ticks of throttle={self.kickstart_value}')
        for i in range(num_throttle_ticks):
            msg = ControlCommand()
            msg.throttle = self.kickstart_value
            msg.steering = 0.0
            msg.brake = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f'  Tick {i+1}/{num_throttle_ticks}: throttle={self.kickstart_value}')
            time.sleep(0.1)

        self.get_logger().info('✅ KICKSTART PULSE COMPLETE, now braking...')

        # Phase 2: Brake
        num_brake_ticks = int(self.brake_duration / 0.1)
        self.get_logger().info(f'[KICKSTART] Phase 2: sending {num_brake_ticks} ticks of brake=0.5')
        for i in range(num_brake_ticks):
            msg = ControlCommand()
            msg.throttle = 0.0
            msg.steering = 0.0
            msg.brake = 0.5
            self.cmd_pub.publish(msg)
            self.get_logger().info(f'  Tick {i+1}/{num_brake_ticks}: brake=0.5')
            time.sleep(0.1)

        # Phase 3: Reset and hand over to autonomous control
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.kickstart_done = True

        self.get_logger().info('✅ KICKSTART COMPLETE!')
        self.get_logger().info('✅ CAR STOPPED')
        self.get_logger().info('🚗 STARTING AUTONOMOUS WAYPOINT FOLLOWING...')
        self.get_logger().info('')

    # ================================================================== #
    # INITIALIZATION
    # ================================================================== #

    def load_waypoints(self):
        """Load waypoints from JSON."""
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                data = json.load(f)
            self.waypoints = [[float(x), float(y)] for x, y in data]
            self.get_logger().info(f'📍 Loaded {len(self.waypoints)} waypoints from waypoints.json')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load waypoints: {e}')
            self.waypoints = []

    # ================================================================== #
    # CALLBACKS & HELPERS
    # ================================================================== #

    def odom_callback(self, msg: Odometry):
        """Update pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def find_closest_waypoint(self):
        """Find closest waypoint within search window."""
        if self.current_pose is None or not self.waypoints:
            return 0

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        min_dist = float('inf')
        best_idx = self.current_idx

        start = max(0, self.current_idx - 20)
        end = min(len(self.waypoints), self.current_idx + 40)

        for i in range(start, end):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist < min_dist:
                min_dist = dist
                best_idx = i

        return best_idx

    def find_target_waypoint(self):
        """Find lookahead target at ~lookahead_distance ahead."""
        if self.current_pose is None or not self.waypoints:
            return None, None

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        closest_idx = self.find_closest_waypoint()
        self.current_idx = closest_idx

        for i in range(closest_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist >= self.lookahead_distance:
                return i, (wx, wy)

        return len(self.waypoints) - 1, self.waypoints[-1]

    def compute_steering_from_angle_error(self, target_x, target_y):
        """
        Aggressive angle-based steering:
        
        angle_error = target_angle - current_yaw (normalized to [-pi, pi])
        steering = steering_multiplier * angle_error
        clamped to [-max_steering, +max_steering]
        
        With steering_multiplier=3.5:
          - angle_error = 0.286 rad (~16°) → steering = 1.0
          - angle_error = 0.143 rad (~8°) → steering = 0.5
          - angle_error = 0.0 rad → steering = 0.0
        
        This is AGGRESSIVE and directly converts angle error to steering command.
        """
        if self.current_pose is None:
            return 0.0

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        dx = target_x - cx
        dy = target_y - cy

        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_yaw

        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # AGGRESSIVE: multiply by steering_multiplier
        steering = self.steering_multiplier * angle_error
        steering = max(-self.max_steering, min(self.max_steering, steering))

        return steering

    # ================================================================== #
    # CONTROL LOOP
    # ================================================================== #

    def follow_waypoints(self):
        """20 Hz: compute steering and throttle."""
        if not self.kickstart_done or self.current_pose is None or not self.waypoints:
            return

        target_idx, target = self.find_target_waypoint()
        if target is None:
            return

        tx, ty = target
        self.steering = self.compute_steering_from_angle_error(tx, ty)

        # Check goal
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        gx, gy = self.waypoints[-1]
        dist_to_goal = math.hypot(gx - cx, gy - cy)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('✅ REACHED GOAL – STOPPING')
            self.throttle = 0.0
            self.steering = 0.0
            self.brake = 0.8
            return

        # Speed scaling on sharp turns (safety)
        turn_mag = abs(self.steering) / max(self.max_steering, 1e-3)
        speed_scale = max(0.4, 1.0 - 0.6 * turn_mag)  # slow on sharp turns
        self.throttle = self.target_speed * speed_scale
        self.brake = 0.0

    def publish_cmd(self):
        """10 Hz: publish ControlCommand."""
        msg = ControlCommand()
        msg.throttle = float(self.throttle)
        msg.steering = float(self.steering)
        msg.brake = float(self.brake)
        self.cmd_pub.publish(msg)

    # ================================================================== #
    # VISUALIZATION
    # ================================================================== #

    def publish_visualization(self):
        """Publish waypoint markers and path."""
        if not self.waypoints or self.current_pose is None:
            return

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        markers = MarkerArray()
        for i, (wx, wy) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'fsds/FSCar'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.25

            dist = math.hypot(wx - cx, wy - cy)
            if i == self.current_idx:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # red = closest
            elif dist < self.lookahead_distance:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # yellow = lookahead
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # green = far
            m.color.a = 0.8
            markers.markers.append(m)

        self.markers_pub.publish(markers)

        path = Path()
        path.header.frame_id = 'fsds/FSCar'
        path.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.waypoints:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(wx)
            p.pose.position.y = float(wy)
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⏹️  Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


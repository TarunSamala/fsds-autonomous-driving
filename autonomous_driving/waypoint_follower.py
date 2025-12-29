#!/usr/bin/env python3
"""
Stanley Controller for FSDS Waypoint Following

Stanley controller = heading error + cross-track error control
Better for car-like vehicles than Pure Pursuit

References:
- F1TENTH Lab 6 (Pure Pursuit variant)
- Stanley Method papers
- Cross-track error minimization
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


class StanleyController(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Stanley Controller Parameters (TUNED FOR TIGHT TRACKING)
        self.declare_parameter('base_speed', 0.02)           # Start VERY slow
        self.declare_parameter('lookahead_distance', 0.6)    # Moderate lookahead
        self.declare_parameter('k_e', 2.0)                   # Cross-track error gain (AGGRESSIVE)
        self.declare_parameter('k_h', 1.5)                   # Heading error gain
        self.declare_parameter('goal_tolerance', 1.0)        # Distance to goal

        self.base_speed = self.get_parameter('base_speed').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.k_e = self.get_parameter('k_e').value           # Tuned for aggressive path tracking
        self.k_h = self.get_parameter('k_h').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # State
        self.waypoints = []
        self.current_pose = None
        self.current_yaw = 0.0
        self.current_idx = 0
        
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        
        self.kickstart_done = False

        # Publishers
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_pub = self.create_publisher(Path, '/follow_path', 10)

        # Subscription
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10
        )

        # Timers
        self.control_timer = self.create_timer(0.1, self.publish_cmd)
        self.follow_timer = self.create_timer(0.02, self.follow_waypoints)  # 50 Hz
        self.vis_timer = self.create_timer(0.5, self.publish_visualization)

        self.load_waypoints()

        print("=" * 80)
        print("🚗 STANLEY CONTROLLER - TIGHT WAYPOINT TRACKING")
        print("=" * 80)
        print(f"✅ Loaded {len(self.waypoints)} waypoints")
        if self.waypoints:
            print(f"   First: ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f})")
        print(f"   Base speed: {self.base_speed} (very slow for accuracy)")
        print(f"   Cross-track error gain (k_e): {self.k_e} (AGGRESSIVE)")
        print(f"   Heading error gain (k_h): {self.k_h}")
        print("")
        print("🚀 KICKSTART INITIATED!")
        print("=" * 80)

        self.kick_thread = threading.Thread(target=self.kickstart_sequence, daemon=True)
        self.kick_thread.start()

    # ======================================================================
    # KICKSTART
    # ======================================================================

    def kickstart_sequence(self):
        """Identical kickstart to keyboard_controller.py"""
        print("[KICKSTART] Throttle 0.2 for 0.5s...")
        for _ in range(5):
            msg = ControlCommand()
            msg.throttle = 0.2
            msg.steering = 0.0
            msg.brake = 0.0
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

        print("[KICKSTART] Brake 0.5 for 1.0s...")
        for _ in range(10):
            msg = ControlCommand()
            msg.throttle = 0.0
            msg.steering = 0.0
            msg.brake = 0.5
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.kickstart_done = True
        print("✅ KICKSTART DONE - Stanley controller starting!")

    # ======================================================================
    # INITIALIZATION
    # ======================================================================

    def load_waypoints(self):
        """Load waypoints from JSON."""
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                data = json.load(f)
            
            self.waypoints = [[float(wp[0]), float(wp[1])] for wp in data]
            print(f"📍 Loaded {len(self.waypoints)} waypoints")
        except Exception as e:
            print(f"❌ Failed to load: {e}")
            self.waypoints = []

    # ======================================================================
    # CALLBACKS
    # ======================================================================

    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    # ======================================================================
    # STANLEY CONTROLLER LOGIC
    # ======================================================================

    def find_closest_waypoint(self):
        """Find index of closest waypoint."""
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

    def find_lookahead_point(self, start_idx):
        """Find waypoint at lookahead_distance ahead."""
        if self.current_pose is None or not self.waypoints:
            return None, None, None

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        for i in range(start_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist >= self.lookahead_distance:
                return i, wx, wy

        # Use last waypoint if none far enough
        if len(self.waypoints) > 0:
            i = len(self.waypoints) - 1
            wx, wy = self.waypoints[i]
            return i, wx, wy
        return None, None, None

    def compute_cross_track_error(self, closest_idx):
        """
        Compute perpendicular distance from vehicle to the line
        connecting closest_idx and closest_idx+1 waypoints.
        
        Positive = left of path, Negative = right of path
        """
        if self.current_pose is None or not self.waypoints or closest_idx >= len(self.waypoints) - 1:
            return 0.0

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        # Get line segment (closest waypoint to next waypoint)
        x1, y1 = self.waypoints[closest_idx]
        x2, y2 = self.waypoints[closest_idx + 1]

        # Distance from point (cx, cy) to line segment
        # Using perpendicular distance formula
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            return 0.0

        # Perpendicular distance (cross product method)
        numerator = abs(dy * cx - dx * cy + x2 * y1 - y2 * x1)
        denominator = math.hypot(dx, dy)
        cte = numerator / denominator

        # Determine sign: is car left or right of path?
        # Use cross product to determine side
        cross = (x2 - x1) * (cy - y1) - (y2 - y1) * (cx - x1)
        if cross < 0:
            cte = -cte

        return cte

    def compute_heading_error(self, target_x, target_y):
        """
        Compute heading error between vehicle heading and direction to target.
        Range: [-pi, pi]
        """
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.current_yaw

        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        return heading_error

    def compute_stanley_steering(self, closest_idx, target_x, target_y):
        """
        Stanley Controller:
        
        steering = heading_error + atan2(cross_track_error, speed)
        
        But since speed is low, use:
        steering = k_h * heading_error + k_e * cross_track_error
        
        This gives aggressive cross-track error correction.
        """
        if self.current_pose is None:
            return 0.0

        # Heading error component
        heading_error = self.compute_heading_error(target_x, target_y)
        heading_term = self.k_h * heading_error

        # Cross-track error component
        cte = self.compute_cross_track_error(closest_idx)
        cte_term = self.k_e * cte

        # Total steering
        steering = heading_term + cte_term
        steering = max(-1.0, min(1.0, steering))

        return steering

    def follow_waypoints(self):
        """
        50 Hz control loop using Stanley Controller.
        """
        if not self.kickstart_done or self.current_pose is None or not self.waypoints:
            return

        # Find where we are
        closest_idx = self.find_closest_waypoint()
        self.current_idx = closest_idx

        # Find lookahead target
        _, target_x, target_y = self.find_lookahead_point(closest_idx)
        if target_x is None:
            self.throttle = 0.0
            self.brake = 0.8
            return

        # Stanley steering control
        self.steering = self.compute_stanley_steering(closest_idx, target_x, target_y)

        # Check goal
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        gx, gy = self.waypoints[-1]
        dist_to_goal = math.hypot(gx - cx, gy - cy)

        if dist_to_goal < self.goal_tolerance:
            print("✅ GOAL REACHED!")
            self.throttle = 0.0
            self.brake = 0.8
            self.steering = 0.0
            return

        # Simple constant speed (can add speed scaling later)
        self.throttle = self.base_speed
        self.brake = 0.0

    def publish_cmd(self):
        """10 Hz: publish control command."""
        msg = ControlCommand()
        msg.throttle = float(self.throttle)
        msg.steering = float(self.steering)
        msg.brake = float(self.brake)
        self.cmd_pub.publish(msg)

    # ======================================================================
    # VISUALIZATION
    # ======================================================================

    def publish_visualization(self):
        """Publish markers and path for RViz."""
        if not self.waypoints or self.current_pose is None:
            return

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        # Waypoint markers
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
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.3

            dist = math.hypot(wx - cx, wy - cy)
            if i == self.current_idx:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # RED: closest
            elif dist < self.lookahead_distance:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # YELLOW: lookahead
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # GREEN: ahead
            m.color.a = 0.8
            markers.markers.append(m)

        self.markers_pub.publish(markers)

        # Path
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
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


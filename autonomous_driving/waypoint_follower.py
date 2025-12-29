#!/usr/bin/env python3
"""
Pure Pursuit Waypoint Follower - Smooth, Stable Racing

Uses Pure Pursuit control law for smooth trajectory following.
No oscillation, handles loops perfectly.
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
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__('pure_pursuit_follower')

        # Pure pursuit parameters
        self.speed = 0.04
        self.lookahead_distance = 0.8  # 80cm lookahead for smooth response
        self.max_steering = 1.0

        # State
        self.waypoints = []
        self.current_pose = None
        self.current_yaw = 0.0
        self.velocity_x = 0.0
        
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.kickstart_done = False
        self.debug_counter = 0
        self.lap_count = 0
        self.last_wp_idx = 0

        # Publishers/Subscriptions
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_pub = self.create_publisher(Path, '/follow_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/testing_only/odom', self.odom_callback, 10)

        # Timers
        self.control_timer = self.create_timer(0.1, self.publish_cmd)
        self.follow_timer = self.create_timer(0.02, self.follow_waypoints)  # 50Hz for smooth control
        self.vis_timer = self.create_timer(1.0, self.publish_visualization)

        self.load_waypoints()

        print("=" * 80)
        print("🏎️  PURE PURSUIT WAYPOINT FOLLOWER")
        print("=" * 80)
        print(f"✅ Loaded {len(self.waypoints)} waypoints")
        print(f"📏 Lookahead distance: {self.lookahead_distance}m")
        print(f"🚗 Speed: {self.speed}")
        print("🚀 KICKSTART INITIATED!")
        print("=" * 80)

        self.kick_thread = threading.Thread(target=self.kickstart_sequence, daemon=True)
        self.kick_thread.start()

    def kickstart_sequence(self):
        print("[KICKSTART] Throttle 0.2 for 0.5s...")
        for _ in range(5):
            msg = ControlCommand()
            msg.throttle = 0.2; msg.steering = 0.0; msg.brake = 0.0
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

        print("[KICKSTART] Brake 0.5 for 1.0s...")
        for _ in range(10):
            msg = ControlCommand()
            msg.throttle = 0.0; msg.steering = 0.0; msg.brake = 0.5
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

        self.kickstart_done = True
        print("✅ PURE PURSUIT ACTIVE!")

    def load_waypoints(self):
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                data = json.load(f)
            self.waypoints = [[float(wp[0]), float(wp[1])] for wp in data if len(wp) >= 2]
            print(f"📍 {len(self.waypoints)} waypoints loaded")
        except Exception as e:
            print(f"❌ Load error: {e}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.velocity_x = msg.twist.twist.linear.x

    def find_closest_waypoint_index(self):
        """Find closest waypoint to current position."""
        if self.current_pose is None or not self.waypoints:
            return 0

        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        min_dist = float('inf')
        closest_idx = 0

        # Search in a window around last known position (more efficient)
        search_range = 100
        start = max(0, self.last_wp_idx - search_range)
        end = min(len(self.waypoints), self.last_wp_idx + search_range)

        for i in range(start, end):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        self.last_wp_idx = closest_idx
        return closest_idx

    def find_lookahead_waypoint(self, start_idx):
        """Find waypoint at lookahead_distance ahead."""
        if self.current_pose is None or not self.waypoints:
            return 0, None, None

        cx, cy = self.current_pose.position.x, self.current_pose.position.y

        # Search forward from start_idx
        best_idx = start_idx
        for i in range(1, len(self.waypoints)):
            idx = (start_idx + i) % len(self.waypoints)
            wx, wy = self.waypoints[idx]
            dist = math.hypot(wx - cx, wy - cy)
            
            if dist >= self.lookahead_distance:
                return idx, wx, wy

        # Fallback: furthest point in range
        return best_idx, self.waypoints[best_idx][0], self.waypoints[best_idx][1]

    def pure_pursuit_control(self, lookahead_x, lookahead_y):
        """
        Pure pursuit steering control.
        
        Geometry: Find the circle passing through car position and lookahead point.
        Steering angle = arctan(2 * L * sin(alpha) / d)
        where:
          - L = lookahead distance
          - alpha = angle to lookahead point
          - d = lookahead distance (distance to lookahead point)
        """
        if self.current_pose is None:
            return 0.0

        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        
        # Vector to lookahead point
        dx = lookahead_x - cx
        dy = lookahead_y - cy
        
        # Distance to lookahead point
        d = math.hypot(dx, dy)
        if d < 0.01:  # Too close
            return 0.0
        
        # Angle to lookahead point in global frame
        alpha = math.atan2(dy, dx)
        
        # Normalize angle error
        angle_error = alpha - self.current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Pure pursuit law: steering = atan(2*L*sin(alpha)/d)
        # Simplified: steering ≈ sin(alpha) for small angles
        steering = math.atan2(2 * self.lookahead_distance * math.sin(angle_error), d)
        
        return max(-self.max_steering, min(self.max_steering, steering))

    def follow_waypoints(self):
        if not self.kickstart_done or self.current_pose is None or not self.waypoints:
            self.throttle = 0.0
            return

        # Find closest waypoint
        closest_idx = self.find_closest_waypoint_index()

        # Find lookahead waypoint
        lookahead_idx, lx, ly = self.find_lookahead_waypoint(closest_idx)
        if lx is None:
            self.throttle = 0.0
            return

        # Compute steering using pure pursuit
        self.steering = self.pure_pursuit_control(lx, ly)
        self.throttle = self.speed
        self.brake = 0.0

        # Lap detection
        if lookahead_idx < self.last_wp_idx - len(self.waypoints) + 50:
            self.lap_count += 1
            print(f"🏁 LAP {self.lap_count + 1} STARTED!")

        # Debug output
        self.debug_counter += 1
        if self.debug_counter % 50 == 0:
            cx, cy = self.current_pose.position.x, self.current_pose.position.y
            dist = math.hypot(lx - cx, ly - cy)
            print(f"Closest:{closest_idx} → Lookahead:{lookahead_idx}({lx:.1f},{ly:.1f}) dist={dist:.2f}m steering={self.steering:.2f}")

    def publish_cmd(self):
        msg = ControlCommand()
        msg.throttle = float(self.throttle)
        msg.steering = float(self.steering)
        msg.brake = float(self.brake)
        self.cmd_pub.publish(msg)

    def publish_visualization(self):
        if not self.waypoints or self.current_pose is None:
            return

        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        markers = MarkerArray()

        closest_idx = self.find_closest_waypoint_index()

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
            m.scale.x = m.scale.y = m.scale.z = 0.25

            dist = math.hypot(wx - cx, wy - cy)
            if i == closest_idx:
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0  # RED: closest
            elif dist < self.lookahead_distance:
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0  # YELLOW: lookahead zone
            else:
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0  # GREEN
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
            path.poses.append(p)
        self.path_pub.publish(path)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PurePursuitFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Stopped")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
Waypoint Follower - SIMPLE REPLAY

Just follow the waypoints exactly as recorded:
- Uses recorded throttle
- Uses recorded steering
- Minimal computation, just position tracking
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


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.waypoints = []
        self.current_idx = 0
        self.current_pose = None
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
        self.follow_timer = self.create_timer(0.05, self.follow_waypoints)
        self.vis_timer = self.create_timer(0.5, self.publish_visualization)

        self.load_waypoints()

        print("=" * 70)
        print("🚗 WAYPOINT FOLLOWER - SIMPLE REPLAY MODE")
        print("=" * 70)
        print(f"✅ Loaded {len(self.waypoints)} waypoints from waypoints.json")
        if self.waypoints:
            wp = self.waypoints[0]
            if len(wp) >= 4:
                print(f"   First WP: x={wp[0]:.2f}, y={wp[1]:.2f}, throttle={wp[2]:.2f}, steering={wp[3]:.3f}")
            else:
                print(f"   First WP: x={wp[0]:.2f}, y={wp[1]:.2f}")
        print("")
        print("🚀 KICKSTART INITIATED!")
        print("=" * 70)

        # Kickstart in background
        self.kick_thread = threading.Thread(target=self.kickstart_sequence, daemon=True)
        self.kick_thread.start()

    def kickstart_sequence(self):
        """Same kickstart as keyboard_controller.py"""
        print("[KICKSTART] Phase 1: throttle 0.2 for 0.5s...")
        for _ in range(5):  # 5 * 0.1s = 0.5s
            msg = ControlCommand()
            msg.throttle = 0.2
            msg.steering = 0.0
            msg.brake = 0.0
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

        print("[KICKSTART] Phase 2: brake 0.5 for 1.0s...")
        for _ in range(10):  # 10 * 0.1s = 1.0s
            msg = ControlCommand()
            msg.throttle = 0.0
            msg.steering = 0.0
            msg.brake = 0.5
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

        print("✅ KICKSTART DONE - Starting replay")
        self.kickstart_done = True

    def load_waypoints(self):
        """Load waypoints [x, y, throttle, steering] or [x, y]"""
        try:
            with open('/workspace/ros2_ws/waypoints.json', 'r') as f:
                data = json.load(f)
            
            self.waypoints = []
            for wp in data:
                if len(wp) == 4:
                    x, y, throttle, steering = wp
                elif len(wp) == 2:
                    x, y = wp
                    throttle, steering = 0.04, 0.0
                else:
                    continue
                self.waypoints.append([float(x), float(y), float(throttle), float(steering)])
            
            print(f"📍 Loaded {len(self.waypoints)} waypoints")
        except Exception as e:
            print(f"❌ Failed to load: {e}")
            self.waypoints = []

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def find_closest_waypoint(self):
        """Find closest waypoint to current position."""
        if self.current_pose is None or not self.waypoints:
            return 0

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        min_dist = float('inf')
        best_idx = self.current_idx

        # Search around current index
        start = max(0, self.current_idx - 10)
        end = min(len(self.waypoints), self.current_idx + 20)

        for i in range(start, end):
            wx, wy, _, _ = self.waypoints[i]
            dist = math.hypot(wx - cx, wy - cy)
            if dist < min_dist:
                min_dist = dist
                best_idx = i

        return best_idx

    def follow_waypoints(self):
        """20 Hz: Just follow the next waypoint in sequence."""
        if not self.kickstart_done or self.current_pose is None or not self.waypoints:
            return

        # Find closest waypoint
        closest_idx = self.find_closest_waypoint()
        self.current_idx = closest_idx

        # Use the NEXT waypoint (look ahead by 1-2 points)
        if self.current_idx + 1 < len(self.waypoints):
            target_idx = self.current_idx + 1
        else:
            target_idx = self.current_idx

        x, y, throttle, steering = self.waypoints[target_idx]

        # Publish recorded values directly
        msg = ControlCommand()
        msg.throttle = float(throttle)
        msg.steering = float(steering)
        msg.brake = 0.0
        self.cmd_pub.publish(msg)

    def publish_cmd(self):
        """10 Hz publish (already done in follow_waypoints, but keep for safety)"""
        pass

    def publish_visualization(self):
        """Show waypoints in RViz."""
        if not self.waypoints or self.current_pose is None:
            return

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        markers = MarkerArray()
        for i, (wx, wy, _, _) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'fsds/FSCar'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.2

            dist = math.hypot(wx - cx, wy - cy)
            if i == self.current_idx:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # red current
            elif dist < 0.5:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # yellow near
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # green
            m.color.a = 0.8
            markers.markers.append(m)

        self.markers_pub.publish(markers)

        # Path
        path = Path()
        path.header.frame_id = 'fsds/FSCar'
        path.header.stamp = self.get_clock().now().to_msg()
        for wx, wy, _, _ in self.waypoints:
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
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


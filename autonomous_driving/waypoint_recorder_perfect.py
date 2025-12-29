#!/usr/bin/env python3
"""
Waypoint Recorder Perfect - FIXED IMPORTS
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path  # ← FIXED: Added Path import
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import json
import math
import time


def euler_from_quaternion(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointRecorderPerfect(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_perfect')
        
        # Recording state
        self.waypoints = []
        self.start_x = None
        self.start_y = None
        self.loop_distance_threshold = 1.0
        self.min_waypoints_before_close = 100
        self.waypoint_spacing = 0.15
        
        self.current_pose = None
        self.current_yaw = 0.0
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/recorded_waypoints', 10)
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10
        )
        
        # Recording control timer
        self.timer = self.create_timer(0.1, self.record_waypoint)
        
        print("=" * 80)
        print("🎯 WAYPOINT RECORDER PERFECT - AUTO-CLOSING")
        print("=" * 80)
        print("📝 Drive around track, it will auto-stop when back near start!")
        print(f"🎯 Loop closes when within {self.loop_distance_threshold}m of start")
        print(f"📏 Waypoint spacing: {self.waypoint_spacing}m")
        print("=" * 80)
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
    def distance_to_start(self):
        if self.start_x is None or self.current_pose is None:
            return float('inf')
        dx = self.current_pose.position.x - self.start_x
        dy = self.current_pose.position.y - self.start_y
        return math.hypot(dx, dy)
        
    def record_waypoint(self):
        if self.current_pose is None:
            return
            
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # First waypoint - set start position
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.waypoints.append([x, y])
            print(f"✅ START recorded: ({x:.2f}, {y:.2f})")
            self.publish_visualization()
            return
            
        # Check if close enough to start to close loop
        dist_to_start = self.distance_to_start()
        if (len(self.waypoints) > self.min_waypoints_before_close and 
            dist_to_start < self.loop_distance_threshold):
            print(f"\n🏁 LOOP DETECTED! dist_to_start={dist_to_start:.2f}m")
            print(f"✅ Recorded {len(self.waypoints)} waypoints")
            self.save_waypoints()
            self.shutdown()
            return
            
        # Add waypoint if far enough from last one
        if not self.waypoints or math.hypot(x - self.waypoints[-1][0], y - self.waypoints[-1][1]) > self.waypoint_spacing:
            self.waypoints.append([x, y])
            self.publish_visualization()
            
            # Status update
            if len(self.waypoints) % 20 == 0:
                print(f"📍 Recorded {len(self.waypoints)} waypoints | Dist to start: {dist_to_start:.2f}m")
                
    def publish_visualization(self):
        markers = MarkerArray()
        
        # Recorded waypoints
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
            m.scale.x = m.scale.y = m.scale.z = 0.2
            
            # Color coding
            if i == 0:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # RED start
            elif self.start_x and math.hypot(wx - self.start_x, wy - self.start_y) < 2.0:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # YELLOW near start
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # GREEN
            m.color.a = 0.8
            markers.markers.append(m)
            
        # Start target zone (big cyan circle)
        if self.start_x:
            target = Marker()
            target.header.frame_id = 'fsds/FSCar'
            target.header.stamp = self.get_clock().now().to_msg()
            target.id = 9999
            target.type = Marker.CYLINDER
            target.action = Marker.ADD
            target.pose.position.x = float(self.start_x)
            target.pose.position.y = float(self.start_y)
            target.pose.position.z = 0.2
            target.scale.x = target.scale.y = 2.0
            target.scale.z = 0.1
            target.color.r, target.color.g, target.color.b = 0.0, 1.0, 1.0  # CYAN
            target.color.a = 0.3
            markers.markers.append(target)
            
        self.markers_pub.publish(markers)
        
        # Path visualization
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
        
    def save_waypoints(self):
        filename = '/workspace/ros2_ws/waypoints.json'
        try:
            with open(filename, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            print(f"💾 SAVED PERFECT LOOP: {filename}")
            print(f"✅ {len(self.waypoints)} waypoints")
            print(f"📏 Final gap: {self.distance_to_start():.2f}m")
        except Exception as e:
            self.get_logger().error(f"❌ Save failed: {e}")
            
    def shutdown(self):
        self.timer.cancel()
        self.get_logger().info("🛑 Recording stopped - perfect loop saved!")


def main(args=None):
    rclpy.init(args=args)
    recorder = WaypointRecorderPerfect()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.save_waypoints()
        print("\n🛑 Interrupted - waypoints saved!")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


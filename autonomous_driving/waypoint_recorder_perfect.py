#!/usr/bin/env python3
"""
Waypoint Recorder with Steering AND Throttle
Records: [[x, y, throttle, steering], ...]
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
import json
import math
import threading


class PerfectWaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_perfect')
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            10
        )
        
        self.sub_control = self.create_subscription(
            ControlCommand,
            '/control_command',
            self.control_callback,
            10
        )
        
        self.waypoints = []
        self.last_waypoint_pos = None
        self.last_throttle = 0.0
        self.last_steering = 0.0
        self.min_distance = 0.15   # Record every 0.15m (more frequent)
        
        print("=" * 70)
        print("🎯 WAYPOINT RECORDER WITH THROTTLE + STEERING")
        print("=" * 70)
        print("Records: [[x, y, throttle, steering], ...]")
        print("")
        print("Commands:")
        print("  'start'  - Start recording")
        print("  'stop'   - Stop recording")
        print("  'save'   - Save to waypoints.json")
        print("  'list'   - Show all waypoints")
        print("  'clear'  - Clear all waypoints")
        print("=" * 70)
        
        self.recording = False
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def odom_callback(self, msg):
        """Record waypoint when distance threshold met."""
        if not self.recording:
            return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.last_waypoint_pos is None:
            self.waypoints.append([x, y, self.last_throttle, self.last_steering])
            self.last_waypoint_pos = [x, y]
            print(f"✅ WP #{len(self.waypoints)}: ({x:.3f}, {y:.3f}) throttle={self.last_throttle:.2f} steering={self.last_steering:.3f}")
        else:
            dist = math.sqrt(
                (x - self.last_waypoint_pos[0])**2 + 
                (y - self.last_waypoint_pos[1])**2
            )
            if dist >= self.min_distance:
                self.waypoints.append([x, y, self.last_throttle, self.last_steering])
                self.last_waypoint_pos = [x, y]
                print(f"✅ WP #{len(self.waypoints)}: ({x:.3f}, {y:.3f}) throttle={self.last_throttle:.2f} steering={self.last_steering:.3f}")

    def control_callback(self, msg: ControlCommand):
        """Update current throttle and steering."""
        self.last_throttle = msg.throttle
        self.last_steering = msg.steering

    def input_loop(self):
        """User input loop."""
        while rclpy.ok():
            cmd = input(">> ").strip().lower()
            
            if cmd == 'start':
                self.recording = True
                self.last_waypoint_pos = None
                print("🔴 RECORDING STARTED - Drive at comfortable speed!")
            
            elif cmd == 'stop':
                self.recording = False
                print(f"⏹️  RECORDING STOPPED - {len(self.waypoints)} waypoints")
            
            elif cmd == 'save':
                if len(self.waypoints) == 0:
                    print("⚠️  No waypoints to save!")
                else:
                    with open('/workspace/ros2_ws/waypoints.json', 'w') as f:
                        json.dump(self.waypoints, f, indent=2)
                    print(f"💾 Saved {len(self.waypoints)} waypoints")
                    print(f"   Format: [[x, y, throttle, steering], ...]")
                    throttles = [w[2] for w in self.waypoints]
                    steerings = [w[3] for w in self.waypoints]
                    print(f"   Throttle range: {min(throttles):.3f} to {max(throttles):.3f}")
                    print(f"   Steering range: {min(steerings):.3f} to {max(steerings):.3f}")
            
            elif cmd == 'list':
                print(f"\nTotal: {len(self.waypoints)} waypoints")
                for i, wp in enumerate(self.waypoints):
                    print(f"  [{i:3d}] x={wp[0]:8.3f} y={wp[1]:8.3f} throttle={wp[2]:5.2f} steering={wp[3]:7.3f}")
                print()
            
            elif cmd == 'clear':
                self.waypoints = []
                self.last_waypoint_pos = None
                print("🗑️  Cleared all waypoints")


def main(args=None):
    rclpy.init(args=args)
    node = PerfectWaypointRecorder()
    rclpy.spin(node)


if __name__ == '__main__':
    main()


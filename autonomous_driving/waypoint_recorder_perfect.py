#!/usr/bin/env python3
"""
Waypoint Recorder with Steering Angles
Records: [[x, y, steering], ...] where steering is from /control_command
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
        self.last_steering = 0.0  # Track current steering command
        self.min_distance = 0.2   # Record waypoint every 0.2m traveled
        
        print("=" * 70)
        print("🎯 PERFECT WAYPOINT RECORDER WITH STEERING")
        print("=" * 70)
        print("Records: [[x, y, steering], ...]")
        print("Where steering is from /control_command (normalized -1 to +1)")
        print("")
        print("Commands:")
        print("  'start'  - Start recording waypoints")
        print("  'stop'   - Stop recording")
        print("  'save'   - Save waypoints to waypoints.json")
        print("  'list'   - Show all waypoints")
        print("  'clear'  - Clear all waypoints")
        print("=" * 70)
        
        self.recording = False
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def odom_callback(self, msg):
        """Called on odometry update. Records waypoint if distance threshold met."""
        if not self.recording:
            return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.last_waypoint_pos is None:
            # First waypoint
            self.waypoints.append([x, y, self.last_steering])
            self.last_waypoint_pos = [x, y]
            print(f"✅ WP #{len(self.waypoints)}: ({x:.3f}, {y:.3f}) steering={self.last_steering:.3f}")
        else:
            # Check if we've traveled far enough
            dist = math.sqrt(
                (x - self.last_waypoint_pos[0])**2 + 
                (y - self.last_waypoint_pos[1])**2
            )
            if dist >= self.min_distance:
                self.waypoints.append([x, y, self.last_steering])
                self.last_waypoint_pos = [x, y]
                print(f"✅ WP #{len(self.waypoints)}: ({x:.3f}, {y:.3f}) steering={self.last_steering:.3f}")

    def control_callback(self, msg: ControlCommand):
        """Called on control command. Updates current steering value."""
        self.last_steering = msg.steering

    def input_loop(self):
        """User input loop for commands."""
        while rclpy.ok():
            cmd = input(">> ").strip().lower()
            
            if cmd == 'start':
                self.recording = True
                self.last_waypoint_pos = None
                self.last_steering = 0.0
                print("🔴 RECORDING STARTED - Drive slowly and smoothly!")
                print("   (steering angles will be captured from /control_command)")
            
            elif cmd == 'stop':
                self.recording = False
                print(f"⏹️  RECORDING STOPPED - {len(self.waypoints)} waypoints recorded")
            
            elif cmd == 'save':
                if len(self.waypoints) == 0:
                    print("⚠️  No waypoints to save!")
                else:
                    with open('/workspace/ros2_ws/waypoints.json', 'w') as f:
                        json.dump(self.waypoints, f, indent=2)
                    print(f"💾 Saved {len(self.waypoints)} waypoints to waypoints.json")
                    print("   Format: [[x, y, steering], ...]")
                    print(f"   Steering range: {min(w[2] for w in self.waypoints):.3f} to {max(w[2] for w in self.waypoints):.3f}")
            
            elif cmd == 'list':
                print(f"\nTotal: {len(self.waypoints)} waypoints")
                for i, wp in enumerate(self.waypoints):
                    if len(wp) == 3:
                        print(f"  [{i:3d}] ({wp[0]:8.3f}, {wp[1]:8.3f}) steering={wp[2]:7.3f}")
                    else:
                        print(f"  [{i:3d}] ({wp[0]:8.3f}, {wp[1]:8.3f})")
                print()
            
            elif cmd == 'clear':
                self.waypoints = []
                self.last_waypoint_pos = None
                self.last_steering = 0.0
                print("🗑️  Cleared all waypoints")
            
            else:
                if cmd:  # Only complain if non-empty
                    print(f"❌ Unknown command: '{cmd}'. Try: start, stop, save, list, clear")


def main(args=None):
    rclpy.init(args=args)
    node = PerfectWaypointRecorder()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

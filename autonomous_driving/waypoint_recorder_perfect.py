#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import math

class PerfectWaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_perfect')
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            10
        )
        
        self.waypoints = []
        self.last_waypoint_pos = None
        self.min_distance = 0.2  # Record waypoint every 0.2m traveled
        
        print("=" * 60)
        print("🎯 PERFECT WAYPOINT RECORDER")
        print("=" * 60)
        print("Commands:")
        print("  'start' - Start recording")
        print("  'stop' - Stop recording")
        print("  'save' - Save waypoints")
        print("  'list' - Show all waypoints")
        print("  'clear' - Clear all waypoints")
        print("=" * 60)
        
        self.recording = False
        
        import threading
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def odom_callback(self, msg):
        if not self.recording:
            return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.last_waypoint_pos is None:
            self.waypoints.append([x, y])
            self.last_waypoint_pos = [x, y]
            print(f"✅ WP #{len(self.waypoints)}: ({x:.3f}, {y:.3f})")
        else:
            dist = math.sqrt((x - self.last_waypoint_pos[0])**2 + (y - self.last_waypoint_pos[1])**2)
            if dist >= self.min_distance:
                self.waypoints.append([x, y])
                self.last_waypoint_pos = [x, y]
                print(f"✅ WP #{len(self.waypoints)}: ({x:.3f}, {y:.3f})")

    def input_loop(self):
        while rclpy.ok():
            cmd = input(">> ").strip().lower()
            
            if cmd == 'start':
                self.recording = True
                self.last_waypoint_pos = None
                print("🔴 RECORDING STARTED - Drive slowly and smoothly!")
            
            elif cmd == 'stop':
                self.recording = False
                print(f"⏹️  RECORDING STOPPED - {len(self.waypoints)} waypoints")
            
            elif cmd == 'save':
                with open('/workspace/ros2_ws/waypoints.json', 'w') as f:
                    json.dump(self.waypoints, f, indent=2)
                print(f"💾 Saved {len(self.waypoints)} waypoints to waypoints.json")
            
            elif cmd == 'list':
                print(f"\nTotal: {len(self.waypoints)} waypoints")
                for i, wp in enumerate(self.waypoints):
                    print(f"  [{i}] ({wp[0]:.3f}, {wp[1]:.3f})")
                print()
            
            elif cmd == 'clear':
                self.waypoints = []
                print("🗑️  Cleared all waypoints")

def main(args=None):
    rclpy.init(args=args)
    node = PerfectWaypointRecorder()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


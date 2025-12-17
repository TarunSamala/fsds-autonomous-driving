#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import json
import os
import math
from datetime import datetime

class AutoWaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_auto')
        
        # Subscriber to odometry
        self.sub_odom = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for waypoints as path
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        
        # Parameters
        self.waypoint_interval = 0.5  # seconds (0.5s interval)
        self.min_distance = 0.3       # meters (minimum distance between waypoints)
        
        # State
        self.waypoints = []
        self.last_waypoint_time = None
        self.last_waypoint_pos = None
        self.recording = False
        self.waypoint_file = '/workspace/ros2_ws/waypoints.json'
        
        # Load existing waypoints if available
        self.load_waypoints()
        
        # Timer for periodic publishing
        self.create_timer(0.1, self.publish_waypoints)
        
        # Input thread for commands
        import threading
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("🎯 Auto Waypoint Recorder Started")
        self.print_help()

    def print_help(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("COMMANDS:")
        self.get_logger().info("  'start' - Start recording waypoints")
        self.get_logger().info("  'stop' - Stop recording")
        self.get_logger().info("  'interval X' - Set interval (e.g., 'interval 0.5')")
        self.get_logger().info("  'list' - Show all waypoints")
        self.get_logger().info("  'save' - Save waypoints to file")
        self.get_logger().info("  'load' - Load waypoints from file")
        self.get_logger().info("  'clear' - Clear all waypoints")
        self.get_logger().info("  'remove N' - Remove waypoint N")
        self.get_logger().info("  'export' - Export as CSV")
        self.get_logger().info("=" * 50)

    def odom_callback(self, msg):
        """Capture waypoints at set intervals."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if not self.recording:
            return
        
        # Check if it's time to capture
        if self.last_waypoint_time is None:
            should_capture = True
        else:
            time_since_last = current_time - self.last_waypoint_time
            should_capture = time_since_last >= self.waypoint_interval
        
        if should_capture:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Check minimum distance
            if self.last_waypoint_pos is not None:
                dist = math.sqrt(
                    (x - self.last_waypoint_pos[0])**2 + 
                    (y - self.last_waypoint_pos[1])**2
                )
                if dist < self.min_distance:
                    return
            
            # Add waypoint
            self.waypoints.append([x, y])
            self.last_waypoint_time = current_time
            self.last_waypoint_pos = [x, y]
            
            self.get_logger().info(
                f"📍 Waypoint {len(self.waypoints)}: ({x:.2f}, {y:.2f})"
            )

    def publish_waypoints(self):
        """Publish waypoints as a Path message."""
        path_msg = Path()
        path_msg.header.frame_id = "fsds/FSCar"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        if len(self.waypoints) > 0:
            self.pub_path.publish(path_msg)

    def input_loop(self):
        """Command input loop."""
        while rclpy.ok():
            try:
                cmd = input(">> ").strip().lower().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'start':
                    self.recording = True
                    self.last_waypoint_time = None
                    self.last_waypoint_pos = None
                    self.get_logger().info("🔴 RECORDING STARTED")
                
                elif cmd[0] == 'stop':
                    self.recording = False
                    self.get_logger().info("⏹️  RECORDING STOPPED")
                    self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")
                
                elif cmd[0] == 'interval' and len(cmd) == 2:
                    self.waypoint_interval = float(cmd[1])
                    self.get_logger().info(f"⏱️  Interval set to {self.waypoint_interval}s")
                
                elif cmd[0] == 'list':
                    if not self.waypoints:
                        self.get_logger().info("No waypoints recorded")
                    else:
                        self.get_logger().info(f"Total: {len(self.waypoints)} waypoints")
                        for i, wp in enumerate(self.waypoints):
                            self.get_logger().info(f"  [{i}] ({wp[0]:.3f}, {wp[1]:.3f})")
                
                elif cmd[0] == 'save':
                    self.save_waypoints()
                    self.get_logger().info("💾 Saved to waypoints.json")
                
                elif cmd[0] == 'load':
                    self.load_waypoints()
                    self.get_logger().info(f"📂 Loaded {len(self.waypoints)} waypoints")
                
                elif cmd[0] == 'clear':
                    self.waypoints = []
                    self.get_logger().info("🗑️  Cleared all waypoints")
                
                elif cmd[0] == 'remove' and len(cmd) == 2:
                    idx = int(cmd[1])
                    if 0 <= idx < len(self.waypoints):
                        removed = self.waypoints.pop(idx)
                        self.get_logger().info(f"❌ Removed waypoint {idx}: {removed}")
                
                elif cmd[0] == 'export':
                    self.export_csv()
                
                elif cmd[0] == 'help':
                    self.print_help()
                
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def save_waypoints(self):
        with open(self.waypoint_file, 'w') as f:
            json.dump(self.waypoints, f, indent=2)

    def load_waypoints(self):
        if os.path.exists(self.waypoint_file):
            with open(self.waypoint_file, 'r') as f:
                self.waypoints = json.load(f)
                self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints from file")

    def export_csv(self):
        """Export waypoints as CSV for analysis."""
        csv_file = f'/workspace/ros2_ws/waypoints_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        with open(csv_file, 'w') as f:
            f.write("index,x,y\n")
            for i, wp in enumerate(self.waypoints):
                f.write(f"{i},{wp[0]:.6f},{wp[1]:.6f}\n")
        self.get_logger().info(f"📊 Exported to {csv_file}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoWaypointRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


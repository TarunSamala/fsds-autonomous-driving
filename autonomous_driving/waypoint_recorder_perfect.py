#!/usr/bin/env python3

"""
Waypoint Recorder Perfect v2 - SLAM-Integrated

- Records clean waypoints from SLAM-corrected odometry
- Auto-closes loop when back at start (within 1.5m)
- Fixes rclpy shutdown race condition
- Compatible with waypoint_follower Pure Pursuit controller
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import json
import math


def euler_from_quaternion(quat):
    """Extract yaw from quaternion"""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointRecorderPerfect(Node):
    """Record clean waypoints for autonomous following"""

    def __init__(self):
        super().__init__('waypoint_recorder_perfect')

        # Recording state
        self.waypoints = []
        self.start_x = None
        self.start_y = None
        self.loop_distance_threshold = 1.5  # 1.5m to close loop
        self.min_waypoints_before_close = 100
        self.waypoint_spacing = 0.2  # 20cm between waypoints
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.recording_active = True
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/recorded_waypoints', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10
        )

        # Recording control timer
        self.timer = self.create_timer(0.1, self.record_waypoint)

        self.get_logger().info("=" * 80)
        self.get_logger().info("🎯 WAYPOINT RECORDER PERFECT - SLAM INTEGRATED")
        self.get_logger().info("=" * 80)
        self.get_logger().info("📝 Drive around track with SLAM running")
        self.get_logger().info(f"🎯 Loop closes when within {self.loop_distance_threshold}m of start")
        self.get_logger().info(f"📏 Waypoint spacing: {self.waypoint_spacing}m")
        self.get_logger().info("=" * 80)

    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def distance_to_start(self) -> float:
        """Calculate distance to start position"""
        if self.start_x is None or self.current_pose is None:
            return float('inf')
        
        dx = self.current_pose.position.x - self.start_x
        dy = self.current_pose.position.y - self.start_y
        return math.hypot(dx, dy)

    def record_waypoint(self):
        """Record waypoints at regular intervals"""
        if self.current_pose is None or not self.recording_active:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # First waypoint - set start position
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.waypoints.append([x, y])
            self.get_logger().info(f"✅ START recorded: ({x:.2f}, {y:.2f})")
            self.publish_visualization()
            return

        # Check if close enough to start to close loop
        dist_to_start = self.distance_to_start()
        
        if (len(self.waypoints) > self.min_waypoints_before_close and
            dist_to_start < self.loop_distance_threshold):
            
            self.get_logger().info(f"\n🏁 LOOP DETECTED! dist_to_start={dist_to_start:.2f}m")
            self.get_logger().info(f"✅ Recorded {len(self.waypoints)} waypoints")
            self.recording_active = False
            self.save_waypoints()
            self.timer.cancel()  # Stop recording
            return

        # Add waypoint if far enough from last one
        if not self.waypoints or math.hypot(
            x - self.waypoints[-1][0], 
            y - self.waypoints[-1][1]
        ) > self.waypoint_spacing:
            
            self.waypoints.append([x, y])
            self.publish_visualization()

        # Status update every 20 waypoints
        if len(self.waypoints) % 20 == 0:
            self.get_logger().info(
                f"📍 Recorded {len(self.waypoints)} waypoints | "
                f"Dist to start: {dist_to_start:.2f}m"
            )

    def publish_visualization(self):
        """Publish waypoints as markers and path for RViz"""
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
            m.scale.x = m.scale.y = m.scale.z = 0.15

            # Color coding
            if i == 0:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # RED start
            elif self.start_x and math.hypot(wx - self.start_x, wy - self.start_y) < 2.0:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # YELLOW near start
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # GREEN

            m.color.a = 0.8
            markers.markers.append(m)

        # Start target zone (cyan circle)
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
        """Save waypoints to JSON file"""
        filename = '/workspace/ros2_ws/waypoints.json'
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            
            self.get_logger().info(f"💾 SAVED PERFECT LOOP: {filename}")
            self.get_logger().info(f"✅ {len(self.waypoints)} waypoints")
            self.get_logger().info(f"📏 Final gap: {self.distance_to_start():.2f}m")
            
        except Exception as e:
            self.get_logger().error(f"❌ Save failed: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        if self.recording_active:
            self.save_waypoints()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = WaypointRecorderPerfect()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("\n🛑 Interrupted - saving waypoints...")
        recorder.save_waypoints()
    except Exception as e:
        recorder.get_logger().error(f"Error: {e}")
    finally:
        recorder.destroy_node()
        
        # Safe shutdown - avoid double shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


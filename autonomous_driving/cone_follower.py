#!/usr/bin/env python3
"""
Cone-Boundary Following Autonomous Controller
Segments track into left/right cone pairs, finds centerline, follows it
Works on closed oval tracks
"""

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from fs_msgs.msg import ControlCommand
from visualization_msgs.msg import MarkerArray, Marker
import struct


class ConeTrackFollower(Node):
    def __init__(self):
        super().__init__('cone_track_follower')
        
        # Parameters
        self.declare_parameter('throttle', 0.035)
        self.declare_parameter('max_steering', 0.42)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('cone_pairing_dist', 2.0)
        self.declare_parameter('track_width_max', 4.0)
        
        self.throttle = self.get_parameter('throttle').value
        self.max_steering = self.get_parameter('max_steering').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.cone_pairing_dist = self.get_parameter('cone_pairing_dist').value
        self.track_width_max = self.get_parameter('track_width_max').value
        
        # State
        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.speed = 0.0
        
        # Cone tracking
        self.left_cones = deque(maxlen=50)
        self.right_cones = deque(maxlen=50)
        self.centerline_path = []
        
        # Debug
        self.last_diag = self.get_clock().now()
        self.frame_count = 0
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/Lidar1',
            self.lidar_callback,
            qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            qos
        )
        
        # Publishers
        self.control_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/centerline_path',
            10
        )
        
        self.left_cones_pub = self.create_publisher(
            MarkerArray,
            '/detected_left_cones',
            10
        )
        
        self.right_cones_pub = self.create_publisher(
            MarkerArray,
            '/detected_right_cones',
            10
        )
        
        self.target_pub = self.create_publisher(
            PointStamped,
            '/target_point',
            10
        )
        
        self.get_logger().info("🏎️  Cone-Track Follower Started")
        self.get_logger().info(f"   Throttle: {self.throttle}")
        self.get_logger().info(f"   Lookahead: {self.lookahead_distance}m")
    
    def odom_callback(self, msg: Odometry):
        """Update robot state"""
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        self.speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )
    
    def lidar_callback(self, msg: PointCloud2):
        """Extract cones and compute steering"""
        points = self.parse_pointcloud2(msg)
        
        # Extract left/right cones
        self.extract_cones(points)
        
        # Build centerline
        self.centerline_path = self.compute_centerline()
        
        # Find target point
        target = self.find_target_point()
        
        if target is not None:
            # Compute steering
            steering = self.compute_steering(target)
            self.publish_visualizations()
        else:
            steering = 0.0
        
        self.publish_control(steering)
        
        self.frame_count += 1
        if (self.get_clock().now() - self.last_diag).nanoseconds > 3e9:
            self.get_logger().info(
                f"🏁 L:{len(self.left_cones)} R:{len(self.right_cones)} "
                f"Path:{len(self.centerline_path)} Speed:{self.speed:.2f}m/s"
            )
            self.last_diag = self.get_clock().now()
    
    def extract_cones(self, points):
        """Separate LiDAR points into left/right cones"""
        self.left_cones.clear()
        self.right_cones.clear()
        
        for x, y, z in points:
            # Cone height filter
            if not (0.05 < z < 0.8):
                continue
            
            # Distance filter
            dist = math.hypot(x, y)
            if dist > 10.0 or dist < 0.2:
                continue
            
            cone = np.array([x, y])
            
            # Separate by Y
            if y > 0.15:  # Left (positive Y)
                self.left_cones.append(cone)
            elif y < -0.15:  # Right (negative Y)
                self.right_cones.append(cone)
    
    def compute_centerline(self):
        """Compute centerline by pairing left/right cones"""
        if len(self.left_cones) < 2 or len(self.right_cones) < 2:
            return []
        
        centerline = []
        
        # Sort cones by X (forward direction)
        left_sorted = sorted(self.left_cones, key=lambda c: c[0])
        right_sorted = sorted(self.right_cones, key=lambda c: c[0])
        
        # Pair cones and compute midpoints
        for left_cone in left_sorted:
            # Find closest right cone at similar X
            distances = [
                abs(left_cone[0] - right_cone[0]) +
                0.5 * abs(left_cone[1] - right_cone[1])
                for right_cone in right_sorted
            ]
            
            if len(distances) == 0:
                continue
            
            closest_idx = np.argmin(distances)
            closest_dist = distances[closest_idx]
            
            # Check if pairing is valid
            if closest_dist > self.cone_pairing_dist:
                continue
            
            right_cone = right_sorted[closest_idx]
            
            # Compute centerline point
            center = (left_cone + right_cone) / 2.0
            
            # Check track width
            track_width = abs(left_cone[1] - right_cone[1])
            if track_width > self.track_width_max:
                continue
            
            centerline.append(center)
        
        return centerline
    
    def find_target_point(self):
        """Find target point ahead at lookahead distance"""
        if len(self.centerline_path) == 0:
            return None
        
        # Find closest point on centerline
        distances = [
            np.linalg.norm(p - self.current_pos)
            for p in self.centerline_path
        ]
        closest_idx = np.argmin(distances)
        
        # Find point at lookahead distance ahead
        target_idx = closest_idx
        for i in range(closest_idx, len(self.centerline_path)):
            dist = np.linalg.norm(
                self.centerline_path[i] - self.current_pos
            )
            if dist >= self.lookahead_distance:
                target_idx = i
                break
        else:
            # Lookahead beyond centerline - extrapolate
            if len(self.centerline_path) > 1:
                target_idx = len(self.centerline_path) - 1
            else:
                return self.centerline_path[0]
        
        return self.centerline_path[target_idx]
    
    def compute_steering(self, target):
        """Stanley steering control"""
        # Vector to target
        to_target = target - self.current_pos
        dist_to_target = np.linalg.norm(to_target)
        
        if dist_to_target < 0.01:
            return 0.0
        
        to_target = to_target / dist_to_target
        
        # Heading error
        target_yaw = math.atan2(to_target[1], to_target[0])
        heading_error = target_yaw - self.current_yaw
        
        # Normalize
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Cross-track error (perpendicular distance to line)
        cte = self.cross_track_error(target)
        
        # Stanley control law
        k_stanley = 0.8
        if self.speed > 0.05:
            steering = heading_error + math.atan2(k_stanley * cte, max(self.speed, 0.1))
        else:
            steering = heading_error
        
        # Clamp
        steering = max(-self.max_steering, min(self.max_steering, steering))
        
        # Publish target
        target_msg = PointStamped()
        target_msg.header.frame_id = "fsds/FSCar"
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.point.x = target[0]
        target_msg.point.y = target[1]
        target_msg.point.z = 0.0
        self.target_pub.publish(target_msg)
        
        return steering
    
    def cross_track_error(self, target):
        """Cross-track error (lateral deviation)"""
        # Simple: Y-distance to target
        return target[1] - self.current_pos[1]
    
    def publish_visualizations(self):
        """Publish cones and centerline for RViz"""
        # Publish centerline path
        if len(self.centerline_path) > 0:
            path_msg = Path()
            path_msg.header.frame_id = "fsds/FSCar"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for point in self.centerline_path:
                pose = PoseStamped()
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
        
        # Publish left cones
        left_markers = MarkerArray()
        for i, cone in enumerate(self.left_cones):
            marker = Marker()
            marker.header.frame_id = "fsds/FSCar"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(cone[0])
            marker.pose.position.y = float(cone[1])
            marker.pose.position.z = 0.15
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            left_markers.markers.append(marker)
        self.left_cones_pub.publish(left_markers)
        
        # Publish right cones
        right_markers = MarkerArray()
        for i, cone in enumerate(self.right_cones):
            marker = Marker()
            marker.header.frame_id = "fsds/FSCar"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(cone[0])
            marker.pose.position.y = float(cone[1])
            marker.pose.position.z = 0.15
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            right_markers.markers.append(marker)
        self.right_cones_pub.publish(right_markers)
    
    def publish_control(self, steering):
        """Publish control"""
        cmd = ControlCommand()
        cmd.steering = steering
        cmd.throttle = self.throttle
        self.control_pub.publish(cmd)
    
    def parse_pointcloud2(self, msg: PointCloud2):
        """Parse PointCloud2"""
        points = []
        field_dict = {f.name: f.offset for f in msg.fields}
        
        if 'x' not in field_dict:
            return points
        
        for i in range(msg.width * msg.height):
            offset = i * msg.point_step
            try:
                x, = struct.unpack('f', msg.data[offset+field_dict['x']:offset+field_dict['x']+4])
                y, = struct.unpack('f', msg.data[offset+field_dict['y']:offset+field_dict['y']+4])
                z, = struct.unpack('f', msg.data[offset+field_dict['z']:offset+field_dict['z']+4])
                
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                    points.append((x, y, z))
            except:
                pass
        
        return points
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Quaternion to yaw"""
        return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def main(args=None):
    rclpy.init(args=args)
    follower = ConeTrackFollower()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        print("\n✋ Controller stopped")
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


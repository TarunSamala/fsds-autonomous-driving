#!/usr/bin/env python3
"""
SLAM Node - De-duplicated Cone Map
Merges cones within 5cm distance to eliminate overlaps.
"""

import numpy as np
from math import atan2, sqrt
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion


def euler_from_quaternion(quat: Quaternion) -> float:
    """Extract yaw angle (radians) from quaternion."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # GLOBAL cone map with de-duplication
        self.cone_map = {}  # key: (x_rounded, y_rounded) → cone_data
        self.merge_distance = 0.05  # 5cm merge distance
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.last_log_count = 0
        
        # Subscriptions
        self.sub_cones = self.create_subscription(
            MarkerArray,
            '/detected_cones',
            self.cones_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self.get_logger().info("📍 Subscribed to /detected_cones")
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self.get_logger().info("📍 Subscribed to /testing_only/odom")
        
        # Publisher
        self.pub_map = self.create_publisher(MarkerArray, '/cone_map', 10)
        self.get_logger().info("📍 Publishing /cone_map")
        
    def odom_callback(self, msg: Odometry):
        """Update car pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
    def cones_callback(self, msg: MarkerArray):
        """Process detected cones with de-duplication."""
        if self.current_pose is None:
            self.get_logger().warn("⚠️  Odometry not ready")
            return
        
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        yaw = self.current_yaw
        
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # Process each detected cone
        for marker in msg.markers:
            cone_x_car = marker.pose.position.x
            cone_y_car = marker.pose.position.y
            
            # Transform to global frame
            cone_x_global = car_x + cone_x_car * cos_yaw - cone_y_car * sin_yaw
            cone_y_global = car_y + cone_x_car * sin_yaw + cone_y_car * cos_yaw
            
            # Check if cone already exists (within merge_distance)
            merged = False
            for existing_key, existing_cone in self.cone_map.items():
                dist = sqrt(
                    (cone_x_global - existing_cone['x'])**2 + 
                    (cone_y_global - existing_cone['y'])**2
                )
                
                # Merge if within 5cm
                if dist < self.merge_distance:
                    # Running average
                    existing_cone['count'] += 1
                    existing_cone['x'] = (
                        (existing_cone['x'] * (existing_cone['count'] - 1) + cone_x_global) / 
                        existing_cone['count']
                    )
                    existing_cone['y'] = (
                        (existing_cone['y'] * (existing_cone['count'] - 1) + cone_y_global) / 
                        existing_cone['count']
                    )
                    merged = True
                    break
            
            # Add new cone if not merged
            if not merged:
                key = (round(cone_x_global, 2), round(cone_y_global, 2))
                if key not in self.cone_map:
                    self.cone_map[key] = {
                        'x': cone_x_global,
                        'y': cone_y_global,
                        'count': 1
                    }
        
        # Publish map
        self._publish_cone_map()
        
    def _publish_cone_map(self):
        """Publish de-duplicated cone map."""
        marker_array = MarkerArray()
        
        for marker_id, (_, cone_data) in enumerate(self.cone_map.items()):
            marker = Marker()
            marker.header.frame_id = "fsds/FSCar"
            marker.header.stamp = self.get_clock().now().to_msg()
            
            marker.ns = "global_cone_map"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(cone_data['x'])
            marker.pose.position.y = float(cone_data['y'])
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.4
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            
            marker_array.markers.append(marker)
        
        self.pub_map.publish(marker_array)
        
        # Log every 50 publishes
        self.last_log_count += 1
        if self.last_log_count % 50 == 0:
            self.get_logger().info(f"🟡 De-duplicated map: {len(self.cone_map)} cones (merge_dist={self.merge_distance*100:.0f}cm)")


def main(args=None):
    rclpy.init(args=args)
    slam_node = SlamNode()
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


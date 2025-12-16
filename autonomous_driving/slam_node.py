#!/usr/bin/env python3
"""
SLAM Node - Accumulates detected cones into a persistent global map.
Transforms cones from car frame → global frame using odometry.
Key fix: self.cone_map NEVER resets, always appends new cones.
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
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    
    # Yaw (z-axis rotation) - THIS IS WHAT WE NEED
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    
    return yaw


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # ========== PERSISTENT GLOBAL MAP ==========
        # Dictionary: (x_rounded, y_rounded) → (x_exact, y_exact)
        # Never reset, only ADD new cones or update existing ones
        self.cone_map = {}
        self.map_lock_enabled = False  # set True after first map built to prevent drift
        
        # Current car pose (from odometry)
        self.current_pose = None
        self.current_yaw = 0.0
        
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
        
        # Publisher for global cone map
        self.pub_map = self.create_publisher(MarkerArray, '/cone_map', 10)
        self.get_logger().info("📍 Publishing /cone_map")
        
    def odom_callback(self, msg: Odometry):
        """Update current car pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        # Disabled: self.get_logger().debug(f"Odom: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}, yaw={np.degrees(self.current_yaw):.1f}°")
        
    def cones_callback(self, msg: MarkerArray):
        """
        Process detected cones, transform to global frame, accumulate into map.
        
        CRITICAL: self.cone_map is NEVER reset. Only new keys are added or existing updated.
        """
        if self.current_pose is None:
            self.get_logger().warn("⚠️  Odometry not ready yet, skipping cones")
            return
        
        # Current car position (global frame)
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        yaw = self.current_yaw
        
        # Process each detected cone (in car frame)
        for marker in msg.markers:
            # Cone position relative to car
            cone_x_car = marker.pose.position.x
            cone_y_car = marker.pose.position.y
            
            # Transform to global frame using 2D rotation matrix
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            
            cone_x_global = car_x + cone_x_car * cos_yaw - cone_y_car * sin_yaw
            cone_y_global = car_y + cone_x_car * sin_yaw + cone_y_car * cos_yaw
            
            # Round to 0.1m precision to merge duplicates
            key = (round(cone_x_global, 1), round(cone_y_global, 1))
            
            # ADD to map if not present, or UPDATE if we have better measurement
            if key not in self.cone_map:
                self.cone_map[key] = {
                    'x': cone_x_global,
                    'y': cone_y_global,
                    'count': 1
                }
            else:
                # Running average to refine position
                old_entry = self.cone_map[key]
                old_entry['count'] += 1
                old_entry['x'] = (old_entry['x'] * (old_entry['count'] - 1) + cone_x_global) / old_entry['count']
                old_entry['y'] = (old_entry['y'] * (old_entry['count'] - 1) + cone_y_global) / old_entry['count']
        
        # Publish the ENTIRE accumulated cone map
        self._publish_cone_map()
        
    def _publish_cone_map(self):
        """Build and publish MarkerArray from the entire self.cone_map."""
        marker_array = MarkerArray()
        marker_array.markers = []
        
        for marker_id, ((key_x, key_y), cone_data) in enumerate(self.cone_map.items()):
            marker = Marker()
            marker.header.frame_id = "fsds/FScar"  # GLOBAL frame (from /testing_only/odom)
            marker.header.stamp = self.get_clock().now().to_msg()
            
            marker.ns = "global_cone_map"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position in global frame (stays fixed)
            marker.pose.position.x = float(cone_data['x'])
            marker.pose.position.y = float(cone_data['y'])
            marker.pose.position.z = 0.1  # height above ground
            marker.pose.orientation.w = 1.0
            
            # Cylinder dimensions
            marker.scale.x = 0.2  # diameter
            marker.scale.y = 0.2  # diameter
            marker.scale.z = 0.4  # height
            
            # Yellow color
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            
            marker_array.markers.append(marker)
        
        self.pub_map.publish(marker_array)
        self.get_logger().info(f"🟡 Published {len(self.cone_map)} cones to /cone_map")


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


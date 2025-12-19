#!/usr/bin/env python3
"""
Odometry Transform Publisher
Reads raw FSDS odometry and broadcasts TF: odom → fsds/FSCar
This enables slam_toolbox to work properly by providing the missing /odom frame
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to FSDS raw odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            10
        )
        
        # Current pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.get_logger().info('✅ Odometry Publisher Started')
        self.get_logger().info('   Broadcasting: odom → fsds/FSCar')
        self.get_logger().info('   Reading from: /testing_only/odom')
    
    def odom_callback(self, msg: Odometry):
        """Extract pose from FSDS odometry and publish as TF"""
        
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to yaw)
        quat = msg.pose.pose.orientation
        # Standard quaternion to yaw conversion
        self.theta = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2)
        )
        
        # Create transform: odom → fsds/FSCar
        transform = TransformStamped()
        
        # Timestamp and frame names
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'          # Parent frame
        transform.child_frame_id = 'fsds/FSCar'     # Child frame
        
        # Translation (position)
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        # Rotation (convert yaw back to quaternion)
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = sy
        transform.transform.rotation.w = cy
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


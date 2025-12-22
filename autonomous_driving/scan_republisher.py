#!/usr/bin/env python3
"""
Converts PointCloud2 to LaserScan for slam_toolbox compatibility.
FSDS publishes /lidar/Lidar1 (PointCloud2)
slam_toolbox expects /scan (LaserScan)
This node bridges them.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import struct
import numpy as np
import math


class ScanRepublisher(Node):
    def __init__(self):
        super().__init__('scan_republisher')
        
        # Subscribe to FSDS PointCloud2
        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/Lidar1',
            self.pointcloud_callback,
            10
        )
        
        # Publish as LaserScan for slam_toolbox
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('✅ Scan Republisher Started')
        self.get_logger().info('   Reading: /lidar/Lidar1 (PointCloud2)')
        self.get_logger().info('   Publishing: /scan (LaserScan)')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Convert PointCloud2 to LaserScan"""
        
        # Extract XYZ points from PointCloud2
        points = self.pointcloud2_to_xyz(msg)
        
        if len(points) == 0:
            return
        
        # Project to 2D for laser scan (use X-Y plane)
        # Calculate range (distance from origin) and angle
        ranges = []
        angles = []
        
        for x, y, z in points:
            # Skip points that are too high or too low (out of scanning plane)
            if abs(z) > 0.5:  # 50cm above/below scanning plane
                continue
            
            r = math.sqrt(x*x + y*y)
            theta = math.atan2(y, x)
            
            if r > 0.1:  # Minimum range
                ranges.append(r)
                angles.append(theta)
        
        if len(ranges) == 0:
            return
        
        # Sort by angle
        sorted_indices = sorted(range(len(angles)), key=lambda i: angles[i])
        sorted_ranges = [ranges[i] for i in sorted_indices]
        sorted_angles = [angles[i] for i in sorted_indices]
        
        # Create LaserScan message
        scan = LaserScan()
        scan.header.frame_id = 'fsds/Lidar1'
        scan.header.stamp = self.get_clock().now().to_msg()
        
        # Set angle parameters
        scan.angle_min = sorted_angles[0] if sorted_angles else -math.pi
        scan.angle_max = sorted_angles[-1] if sorted_angles else math.pi
        scan.angle_increment = 0.01  # ~0.57 degrees
        
        # Set range parameters
        scan.range_min = 0.1
        scan.range_max = 50.0
        scan.ranges = sorted_ranges
        
        # Time info
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        
        self.pub.publish(scan)
    
    def pointcloud2_to_xyz(self, msg: PointCloud2):
        """Extract X,Y,Z from PointCloud2"""
        if msg.point_step < 12:
            return []
        
        points = []
        n_points = len(msg.data) // msg.point_step
        
        offset_x = 0
        offset_y = 4
        offset_z = 8
        
        for i in range(n_points):
            base = i * msg.point_step
            try:
                x = struct.unpack_from('f', msg.data, base + offset_x)[0]
                y = struct.unpack_from('f', msg.data, base + offset_y)[0]
                z = struct.unpack_from('f', msg.data, base + offset_z)[0]
                points.append((x, y, z))
            except:
                continue
        
        return points


def main(args=None):
    rclpy.init(args=args)
    node = ScanRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


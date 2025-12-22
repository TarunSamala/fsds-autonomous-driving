#!/usr/bin/env python3
"""
Fixed PointCloud2 to LaserScan converter for FSDS
Properly creates uniform LaserScan with consistent angle bounds
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
        
        # Fixed scan parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 2 * math.pi / 98  # 98 rays
        self.num_rays = 98
        
        self.get_logger().info('✅ Scan Republisher Started')
        self.get_logger().info('   Reading: /lidar/Lidar1 (PointCloud2)')
        self.get_logger().info('   Publishing: /scan (LaserScan with 98 rays)')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Convert PointCloud2 to LaserScan with FIXED 98 rays"""
        
        # Extract XYZ points
        points = self.pointcloud2_to_xyz(msg)
        
        if len(points) == 0:
            return
        
        # Initialize ranges array with max range (50m)
        ranges = np.full(self.num_rays, 50.0, dtype=np.float32)
        
        # Project 3D points to 2D polar coordinates
        for x, y, z in points:
            # Skip points outside scanning plane
            if abs(z) > 0.5:
                continue
            
            # Calculate range and angle
            r = math.sqrt(x*x + y*y)
            
            # Skip too close or too far
            if r < 0.1 or r > 50.0:
                continue
            
            theta = math.atan2(y, x)
            
            # Map angle to ray index
            # Normalize theta to [angle_min, angle_max]
            if theta < self.angle_min:
                theta += 2 * math.pi
            if theta > self.angle_max:
                theta -= 2 * math.pi
            
            # Find closest ray
            ray_idx = int((theta - self.angle_min) / self.angle_increment)
            ray_idx = max(0, min(self.num_rays - 1, ray_idx))
            
            # Keep minimum range for each ray
            if r < ranges[ray_idx]:
                ranges[ray_idx] = r
        
        # Create LaserScan message
        scan = LaserScan()
        scan.header.frame_id = 'fsds/Lidar1'
        scan.header.stamp = self.get_clock().now().to_msg()
        
        # Fixed angle parameters
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        
        # Range parameters
        scan.range_min = 0.1
        scan.range_max = 50.0
        
        # Set ranges (98 rays, exactly)
        scan.ranges = ranges.tolist()
        
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
                
                # Skip NaN/Inf values
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    continue
                if math.isinf(x) or math.isinf(y) or math.isinf(z):
                    continue
                
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


#!/usr/bin/env python3
"""
Working Grid Mapper for FSDS - Actually updates the occupancy grid
Uses occupancy grid from sensor_msgs for direct visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
import math


class GridMapperFixed(Node):
    def __init__(self):
        super().__init__('grid_mapper')
        
        # Grid parameters
        self.resolution = 0.05  # 5cm per cell
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.grid_size = 201  # 201x201 grid = ~10m x 10m
        
        # Initialize grid (0-100, -1 unknown)
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)
        
        # Occupancy probability (log-odds for stability)
        self.log_odds = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        
        # Update parameters
        self.p_occ = 0.65      # Probability of occupied when hit
        self.p_free = 0.35     # Probability of free when miss
        
        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        
        self.get_logger().info('✅ Grid Mapper (Fixed) Started')
        self.get_logger().info(f'   Grid: {self.grid_size}x{self.grid_size} cells')
        self.get_logger().info(f'   Resolution: {self.resolution}m per cell')
        self.get_logger().info('   Publishing: /occupancy_grid')
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan and update grid"""
        
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        
        # Update grid for each ray
        for angle, range_val in zip(angles, ranges):
            
            # Skip invalid ranges
            if range_val <= msg.range_min or range_val >= msg.range_max:
                continue
            
            # Ray endpoints
            end_x = range_val * math.cos(angle)
            end_y = range_val * math.sin(angle)
            
            # Bresenham raytracing from origin to endpoint
            self.trace_ray(0, 0, end_x, end_y, occupied=True)
        
        # Publish grid
        self.publish_grid(msg.header.stamp)
    
    def trace_ray(self, x0, y0, x1, y1, occupied=True):
        """Bresenham line algorithm - mark free cells and endpoint"""
        
        # Convert to grid indices
        x0_grid = self.world_to_grid_x(x0)
        y0_grid = self.world_to_grid_y(y0)
        x1_grid = self.world_to_grid_x(x1)
        y1_grid = self.world_to_grid_y(y1)
        
        # Bresenham line
        points = self.bresenham_line(x0_grid, y0_grid, x1_grid, y1_grid)
        
        # Mark free cells (all but last point)
        for i, (x, y) in enumerate(points[:-1]):
            if self.in_bounds(x, y):
                self.update_cell(x, y, occupied=False)
        
        # Mark occupied cell (last point)
        if len(points) > 0:
            x, y = points[-1]
            if self.in_bounds(x, y) and occupied:
                self.update_cell(x, y, occupied=True)
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def world_to_grid_x(self, x):
        """Convert world X to grid index"""
        grid_x = int((x - self.origin_x) / self.resolution + self.grid_size / 2)
        return grid_x
    
    def world_to_grid_y(self, y):
        """Convert world Y to grid index"""
        grid_y = int((y - self.origin_y) / self.resolution + self.grid_size / 2)
        return grid_y
    
    def in_bounds(self, x, y):
        """Check if grid cell is within bounds"""
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size
    
    def update_cell(self, x, y, occupied):
        """Update cell using log-odds"""
        if occupied:
            self.log_odds[y, x] += math.log(self.p_occ / (1 - self.p_occ))
        else:
            self.log_odds[y, x] += math.log((1 - self.p_free) / self.p_free)
        
        # Clamp log-odds
        self.log_odds[y, x] = np.clip(self.log_odds[y, x], -2.0, 2.0)
        
        # Convert to probability
        prob = 1.0 / (1.0 + math.exp(-self.log_odds[y, x]))
        
        # Convert to 0-100 occupancy value
        self.grid[y, x] = int(prob * 100)
    
    def publish_grid(self, timestamp):
        """Publish occupancy grid"""
        
        msg = OccupancyGrid()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        
        # Metadata
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_x - (self.grid_size / 2) * self.resolution
        msg.info.origin.position.y = self.origin_y - (self.grid_size / 2) * self.resolution
        
        # Grid data (row-major, origin at bottom-left)
        data = []
        for iy in range(self.grid_size):
            for ix in range(self.grid_size):
                # Flip Y axis for ROS convention
                data.append(int(self.grid[self.grid_size - 1 - iy, ix]))
        
        msg.data = data
        self.grid_pub.publish(msg)
        
        # Log status
        occupied = np.sum(self.grid > 50)
        free = np.sum(self.grid >= 0) - np.sum(self.grid > 50)
        self.get_logger().info(
            f'Grid: occupied={occupied}, free={free}, unknown={np.sum(self.grid == -1)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GridMapperFixed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


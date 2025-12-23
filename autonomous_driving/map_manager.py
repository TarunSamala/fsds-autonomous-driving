#!/usr/bin/env python3
"""
Map Persistence System for Autonomous Driving
Saves and loads cone map + occupancy grid for replay
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import pickle
import os
from pathlib import Path


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        # Map storage directory
        self.map_dir = Path.home() / 'ros2_maps'
        self.map_dir.mkdir(parents=True, exist_ok=True)
        
        self.declare_parameter('map_name', 'track_map')
        self.map_name = self.get_parameter('map_name').value
        
        # Storage
        self.occupancy_grid = None
        self.cone_markers = []
        self.is_recording = False
        
        # Subscriptions
        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.grid_callback,
            10
        )
        
        self.cones_sub = self.create_subscription(
            MarkerArray,
            '/cone_map',
            self.cones_callback,
            10
        )
        
        # Publishers
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map_loaded', 10)
        self.cones_pub = self.create_publisher(MarkerArray, '/cones_loaded', 10)
        
        self.get_logger().info('✅ Map Manager Started')
        self.get_logger().info(f'   Map directory: {self.map_dir}')
        self.get_logger().info(f'   Available maps: {self.list_maps()}')
    
    def grid_callback(self, msg: OccupancyGrid):
        """Store occupancy grid"""
        self.occupancy_grid = msg
        self.get_logger().debug(f'Grid updated: {msg.info.width}x{msg.info.height}')
    
    def cones_callback(self, msg: MarkerArray):
        """Store cone markers"""
        self.cone_markers = msg.markers
        self.get_logger().debug(f'Cones updated: {len(msg.markers)} markers')
    
    def save_map(self):
        """Save current map to disk"""
        if self.occupancy_grid is None or len(self.cone_markers) == 0:
            self.get_logger().warn('Cannot save: grid or cones missing')
            return False
        
        map_file = self.map_dir / f'{self.map_name}.pkl'
        data = {
            'occupancy_grid': self.occupancy_grid,
            'cone_markers': self.cone_markers,
            'timestamp': self.get_clock().now()
        }
        
        try:
            with open(map_file, 'wb') as f:
                pickle.dump(data, f)
            self.get_logger().info(f'✅ Map saved: {map_file}')
            return True
        except Exception as e:
            self.get_logger().error(f'Save failed: {e}')
            return False
    
    def load_map(self):
        """Load map from disk"""
        map_file = self.map_dir / f'{self.map_name}.pkl'
        
        if not map_file.exists():
            self.get_logger().error(f'Map not found: {map_file}')
            return False
        
        try:
            with open(map_file, 'rb') as f:
                data = pickle.load(f)
            
            self.occupancy_grid = data['occupancy_grid']
            self.cone_markers = data['cone_markers']
            
            # Publish loaded maps
            self.grid_pub.publish(self.occupancy_grid)
            
            marker_array = MarkerArray()
            marker_array.markers = self.cone_markers
            self.cones_pub.publish(marker_array)
            
            self.get_logger().info(f'✅ Map loaded: {map_file}')
            self.get_logger().info(f'   Cones: {len(self.cone_markers)}')
            return True
        except Exception as e:
            self.get_logger().error(f'Load failed: {e}')
            return False
    
    def list_maps(self):
        """List available saved maps"""
        maps = list(self.map_dir.glob('*.pkl'))
        return [m.stem for m in maps]
    
    def delete_map(self, name):
        """Delete saved map"""
        map_file = self.map_dir / f'{name}.pkl'
        if map_file.exists():
            map_file.unlink()
            self.get_logger().info(f'Map deleted: {name}')
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


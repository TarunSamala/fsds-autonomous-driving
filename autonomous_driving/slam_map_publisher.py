#!/usr/bin/env python3

"""
SLAM Map Publisher - Load & Republish Saved Maps

- Loads pre-computed cone map from JSON
- Publishes to /cone_map_v2 for visualization
- No SLAM computation - just visualization
- Allows waypoint recording without map overwriting
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
import json
from pathlib import Path


class SlamMapPublisher(Node):
    """Load and republish pre-saved SLAM maps"""

    def __init__(self):
        super().__init__('slam_map_publisher')

        # Declare parameter for map file
        self.declare_parameter('map_file', '/tmp/fsds_slam_map.json')
        self.declare_parameter('map_frame', 'fsds/FSCar')
        
        self.map_file = self.get_parameter('map_file').value
        self.map_frame = self.get_parameter('map_frame').value

        # Publisher
        self.pub_map = self.create_publisher(MarkerArray, '/cone_map_v2', 10)
        
        # Load map from file
        self.landmarks = self.load_map(self.map_file)
        
        if not self.landmarks:
            self.get_logger().error(f"❌ Failed to load map from {self.map_file}")
            return
        
        self.get_logger().info(f"✅ Loaded {len(self.landmarks)} landmarks from {self.map_file}")
        
        # Publish map periodically (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.publish_count = 0

    def load_map(self, filepath: str) -> list:
        """Load cone landmarks from saved JSON file"""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            landmarks = data.get('landmarks', [])
            lap_count = data.get('lap_count', 0)
            
            self.get_logger().info(f"📊 Map info: {len(landmarks)} cones, {lap_count} laps")
            return landmarks
            
        except FileNotFoundError:
            self.get_logger().error(f"❌ Map file not found: {filepath}")
            return []
        except json.JSONDecodeError:
            self.get_logger().error(f"❌ Invalid JSON in {filepath}")
            return []
        except Exception as e:
            self.get_logger().error(f"❌ Error loading map: {e}")
            return []

    def publish_map(self):
        """Publish landmarks as RViz markers"""
        marker_array = MarkerArray()

        for landmark in self.landmarks:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "slam_cones"
            marker.id = landmark.get('id', 0)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = float(landmark.get('x', 0.0))
            marker.pose.position.y = float(landmark.get('y', 0.0))
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0

            # Size
            marker.scale.x = 0.23
            marker.scale.y = 0.23
            marker.scale.z = 0.4

            # Color based on confidence
            observations = landmark.get('observations', 1)
            confidence = landmark.get('confidence', 0.5)

            marker.color.a = float(confidence)
            if observations < 5:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0  # Orange (new)
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # Green (high conf)

            marker_array.markers.append(marker)

        self.pub_map.publish(marker_array)
        self.publish_count += 1

        # Status update every 10 publishes
        if self.publish_count % 10 == 0:
            self.get_logger().debug(f"📡 Published {len(marker_array.markers)} markers")


def main(args=None):
    rclpy.init(args=args)
    publisher = SlamMapPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info("🛑 Map publisher stopped")
    finally:
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


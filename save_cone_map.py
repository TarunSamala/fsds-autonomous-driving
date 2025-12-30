#!/usr/bin/env python3
"""
Save SLAM Cone Map - One-time operation
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import json
import time


class ConeMapSaver(Node):
    def __init__(self):
        super().__init__('cone_map_saver')
        self.cones = []
        self.sub = self.create_subscription(
            MarkerArray, '/cone_map', self.callback, 10
        )
        self.timer = self.create_timer(2.0, self.save_map)
        print("💾 Waiting for /cone_map... (drive 1 lap first)")

    def callback(self, msg):
        self.cones = []
        for marker in msg.markers:
            if marker.ns == "global_cone_map":
                self.cones.append({
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y
                })

    def save_map(self):
        if len(self.cones) > 50:  # Good map
            filename = '/workspace/ros2_ws/cone_map.json'
            with open(filename, 'w') as f:
                json.dump(self.cones, f, indent=2)
            print(f"✅ SAVED {len(self.cones)} cones to {filename}")
            rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    saver = ConeMapSaver()
    rclpy.spin(saver)


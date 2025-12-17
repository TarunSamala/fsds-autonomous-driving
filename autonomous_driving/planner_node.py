#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        self.sub_map = self.create_subscription(
            MarkerArray,
            '/cone_map',
            self.map_callback,
            10
        )
        
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        
        self.get_logger().info("✅ Planner Node Started (Improved). Waiting for /cone_map...")

    def map_callback(self, msg):
        if len(msg.markers) < 20:
            return

        # 1. Extract cone positions
        points = np.array([[m.pose.position.x, m.pose.position.y] for m in msg.markers])
        
        # 2. Find centroid
        centroid = np.mean(points, axis=0)
        
        # 3. Compute distances from centroid
        distances = np.linalg.norm(points - centroid, axis=1)
        
        # 4. Split into INNER and OUTER cones (track boundaries)
        # Use median distance as threshold
        median_dist = np.median(distances)
        inner_cones = points[distances < median_dist]
        outer_cones = points[distances >= median_dist]
        
        if len(inner_cones) < 5 or len(outer_cones) < 5:
            # Fallback: just use all cones sorted by angle
            angles = np.arctan2(points[:, 1] - centroid[1], points[:, 0] - centroid[0])
            sorted_indices = np.argsort(angles)
            sorted_points = points[sorted_indices]
        else:
            # Sort each boundary by angle
            inner_angles = np.arctan2(inner_cones[:, 1] - centroid[1], 
                                     inner_cones[:, 0] - centroid[0])
            outer_angles = np.arctan2(outer_cones[:, 1] - centroid[1], 
                                     outer_cones[:, 0] - centroid[0])
            
            inner_sorted = inner_cones[np.argsort(inner_angles)]
            outer_sorted = outer_cones[np.argsort(outer_angles)]
            
            # Compute midpoints between inner and outer at similar angles
            # Resample to match counts
            num_points = min(len(inner_sorted), len(outer_sorted))
            
            if num_points < 10:
                return
            
            # Simple approach: average inner and outer boundaries
            sorted_points = (inner_sorted[:num_points] + outer_sorted[:num_points]) / 2.0
        
        # 5. Close the loop
        sorted_points = np.vstack([sorted_points, sorted_points[0]])
        
        # 6. Smooth using linear interpolation
        smooth_path = self.simple_spline(sorted_points, num_points=200)
        
        # 7. Publish Path
        path_msg = Path()
        path_msg.header.frame_id = "fsds/FSCar"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for pt in smooth_path:
            pose = PoseStamped()
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.pub_path.publish(path_msg)
        self.get_logger().info(f"📍 Published path with {len(smooth_path)} points")

    def simple_spline(self, points, num_points=200):
        """Smooth path using linear interpolation."""
        if len(points) < 3:
            return points
        
        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        distances = np.insert(distances, 0, 0)
        cumsum = np.cumsum(distances)
        t = cumsum / cumsum[-1]
        t_new = np.linspace(0, 1, num_points)
        
        x_smooth = np.interp(t_new, t, points[:, 0])
        y_smooth = np.interp(t_new, t, points[:, 1])
        
        return np.column_stack([x_smooth, y_smooth])

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


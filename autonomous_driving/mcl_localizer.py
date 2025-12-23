#!/usr/bin/env python3
"""
Monte Carlo Localizer for FSDS
Particle filter with 250 particles for real-time localization
F1TENTH competition-standard implementation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid


class Particle:
    """Represents single particle in filter"""
    def __init__(self, x=0, y=0, theta=0, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class MCLLocalizer(Node):
    def __init__(self):
        super().__init__('mcl_localizer')
        
        # Parameters
        self.num_particles = 250
        self.grid_resolution = 0.05
        self.update_rate = 10  # Hz
        self.motion_noise_x = 0.01      # 1cm std dev
        self.motion_noise_y = 0.01
        self.motion_noise_theta = 0.05  # 0.05 rad std dev
        self.sensor_noise = 0.5         # 50cm std dev
        
        # Initialize particles
        self.particles = [Particle(x=np.random.normal(0, 0.5),
                                  y=np.random.normal(0, 0.5),
                                  theta=np.random.uniform(-math.pi, math.pi))
                         for _ in range(self.num_particles)]
        
        # Grid storage
        self.grid = {}
        self.grid_resolution = 0.05
        
        # Previous odometry
        self.prev_odom = None
        self.best_pose = None
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/testing_only/odom', self.odom_callback, 10)
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/occupancy_grid', self.grid_callback, 10)
        
        self.get_logger().info('✅ MCL Localizer Started')
        self.get_logger().info(f'   Particles: {self.num_particles}')
        self.get_logger().info('   Publishing: /tf (map → odom)')
    
    def odom_callback(self, msg: Odometry):
        """Apply motion model using odometry"""
        if self.prev_odom is None:
            self.prev_odom = msg
            return
        
        # Calculate delta movement
        dx = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        
        # Extract previous orientation
        q = self.prev_odom.pose.pose.orientation
        prev_theta = math.atan2(2*(q.w*q.z + q.x*q.y),
                               1 - 2*(q.y**2 + q.z**2))
        
        # Extract current orientation
        q = msg.pose.pose.orientation
        curr_theta = math.atan2(2*(q.w*q.z + q.x*q.y),
                               1 - 2*(q.y**2 + q.z**2))
        dtheta = curr_theta - prev_theta
        
        # Normalize angle
        while dtheta > math.pi:
            dtheta -= 2*math.pi
        while dtheta < -math.pi:
            dtheta += 2*math.pi
        
        # Apply motion to particles with noise
        for particle in self.particles:
            noise_x = np.random.normal(0, self.motion_noise_x * abs(dx) + 0.01)
            noise_y = np.random.normal(0, self.motion_noise_y * abs(dy) + 0.01)
            noise_theta = np.random.normal(0, self.motion_noise_theta * abs(dtheta) + 0.01)
            
            particle.x += dx + noise_x
            particle.y += dy + noise_y
            particle.theta += dtheta + noise_theta
            
            # Normalize angle
            particle.theta = math.atan2(math.sin(particle.theta),
                                       math.cos(particle.theta))
        
        self.prev_odom = msg
    
    def scan_callback(self, msg: LaserScan):
        """Update particle weights based on scan"""
        if len(self.grid) == 0:
            self.get_logger().debug('Waiting for grid...')
            return
        
        # Compute weight for each particle
        for particle in self.particles:
            particle.weight = self.compute_scan_likelihood(particle, msg)
        
        # Normalize weights
        total_weight = sum(p.weight for p in self.particles)
        if total_weight > 0:
            for particle in self.particles:
                particle.weight /= total_weight
        else:
            # Reset if all weights zero
            for particle in self.particles:
                particle.weight = 1.0 / self.num_particles
        
        # Resample particles (low variance)
        self.resample_particles()
        
        # Publish pose estimate
        self.publish_pose_estimate()
        
        # Publish TF
        self.publish_tf()
    
    def compute_scan_likelihood(self, particle, scan):
        """Compute likelihood of scan given particle pose"""
        likelihood = 1.0
        max_likelihood = 0.0
        
        # Sample rays at regular intervals
        n_samples = 20
        indices = np.linspace(0, len(scan.ranges)-1, n_samples, dtype=int)
        
        for idx in indices:
            angle = scan.angle_min + idx * scan.angle_increment
            range_val = scan.ranges[idx]
            
            if range_val < scan.range_min or range_val > scan.range_max:
                continue
            
            # Transform to global frame
            global_x = particle.x + range_val * math.cos(particle.theta + angle)
            global_y = particle.y + range_val * math.sin(particle.theta + angle)
            
            # Get grid occupancy at this point
            grid_x = int(global_x / self.grid_resolution)
            grid_y = int(global_y / self.grid_resolution)
            
            occupancy = self.grid.get((grid_x, grid_y), -1)
            
            if occupancy >= 50:
                # High probability of being occupied - good match
                likelihood *= math.exp(-(occupancy - 70)**2 / (2 * self.sensor_noise**2))
                max_likelihood += 1.0
            else:
                # Low probability - penalize
                likelihood *= 0.1
        
        # Normalize by number of samples
        if max_likelihood > 0:
            likelihood = likelihood ** (1.0 / max_likelihood)
        
        return max(likelihood, 0.001)
    
    def resample_particles(self):
        """Low-variance resampling"""
        new_particles = []
        weights = np.array([p.weight for p in self.particles])
        
        # Cumulative sum
        cumsum = np.cumsum(weights)
        
        # Random start
        r = np.random.uniform(0, 1.0 / self.num_particles)
        
        # Resample
        j = 0
        for i in range(self.num_particles):
            u = r + i / self.num_particles
            while u > cumsum[j] and j < len(cumsum) - 1:
                j += 1
            
            p = self.particles[j]
            new_particles.append(Particle(p.x, p.y, p.theta, 1.0/self.num_particles))
        
        self.particles = new_particles
    
    def grid_callback(self, msg: OccupancyGrid):
        """Update grid representation"""
        self.grid = {}
        
        # Convert OccupancyGrid message to dict
        for iy in range(msg.info.height):
            for ix in range(msg.info.width):
                idx = iy * msg.info.width + ix
                if idx < len(msg.data):
                    value = msg.data[idx]
                    if value >= 0:
                        # Convert back to grid coordinates
                        gx = int((msg.info.origin.position.x + ix * msg.info.resolution) 
                                 / self.grid_resolution)
                        gy = int((msg.info.origin.position.y + iy * msg.info.resolution)
                                 / self.grid_resolution)
                        self.grid[(gx, gy)] = value
    
    def publish_pose_estimate(self):
        """Publish best pose estimate from particles"""
        if len(self.particles) == 0:
            return
            
        # Weighted mean
        mean_x = sum(p.x * p.weight for p in self.particles)
        mean_y = sum(p.y * p.weight for p in self.particles)
        mean_theta = sum(p.theta * p.weight for p in self.particles)
        
        # Covariance
        cov_x = sum(p.weight * (p.x - mean_x)**2 for p in self.particles)
        cov_y = sum(p.weight * (p.y - mean_y)**2 for p in self.particles)
        
        # Store best pose
        self.best_pose = (mean_x, mean_y, mean_theta)
    
    def publish_tf(self):
        """Publish map → odom transform"""
        if self.best_pose is None:
            return
        
        x, y, theta = self.best_pose
        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = sy
        transform.transform.rotation.w = cy
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = MCLLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


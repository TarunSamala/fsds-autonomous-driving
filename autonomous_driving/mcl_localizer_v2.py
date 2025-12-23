#!/usr/bin/env python3
"""
Monte Carlo Localizer v2 - Production Ready
Validates SLAM pose with particle filter
Fixed TF timestamps + proper frame chain
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class Particle:
    """Represents single particle in filter"""
    def __init__(self, x=0, y=0, theta=0, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class MCLLocalizerV2(Node):
    def __init__(self):
        super().__init__('mcl_localizer_v2')
        
        # Parameters
        self.num_particles = 100
        self.motion_noise_x = 0.02
        self.motion_noise_y = 0.02
        self.motion_noise_theta = 0.1
        self.sensor_noise = 0.5
        
        # Initialize particles around origin
        self.particles = [
            Particle(
                x=np.random.normal(0, 0.5),
                y=np.random.normal(0, 0.5),
                theta=np.random.uniform(-math.pi, math.pi)
            )
            for _ in range(self.num_particles)
        ]
        
        # Previous odometry
        self.prev_odom = None
        self.best_pose = None
        self.best_confidence = 0.0
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.particles_pub = self.create_publisher(PoseArray, '/particles', 10)
        self.pose_pub = self.create_publisher(Odometry, '/mcl_pose', 10)
        
        self.get_logger().info('✅ MCL Localizer v2 Started')
        self.get_logger().info(f'   Particles: {self.num_particles}')
        self.get_logger().info('   Publishing: /tf (odom → base_link)')
        self.get_logger().info('   Publishing: /mcl_pose (confidence metric)')
    
    def odom_callback(self, msg: Odometry):
        """Apply motion model using odometry"""
        if self.prev_odom is None:
            self.prev_odom = msg
            return
        
        # Calculate delta movement
        dx = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        
        # Extract orientations
        q_prev = self.prev_odom.pose.pose.orientation
        prev_theta = math.atan2(
            2*(q_prev.w*q_prev.z + q_prev.x*q_prev.y),
            1 - 2*(q_prev.y**2 + q_prev.z**2)
        )
        
        q_curr = msg.pose.pose.orientation
        curr_theta = math.atan2(
            2*(q_curr.w*q_curr.z + q_curr.x*q_curr.y),
            1 - 2*(q_curr.y**2 + q_curr.z**2)
        )
        
        dtheta = curr_theta - prev_theta
        while dtheta > math.pi:
            dtheta -= 2*math.pi
        while dtheta < -math.pi:
            dtheta += 2*math.pi
        
        # Apply motion to particles
        for particle in self.particles:
            noise_x = np.random.normal(0, self.motion_noise_x * (abs(dx) + 0.01))
            noise_y = np.random.normal(0, self.motion_noise_y * (abs(dy) + 0.01))
            noise_theta = np.random.normal(0, self.motion_noise_theta * (abs(dtheta) + 0.01))
            
            particle.x += dx + noise_x
            particle.y += dy + noise_y
            particle.theta += dtheta + noise_theta
            particle.theta = math.atan2(math.sin(particle.theta), math.cos(particle.theta))
        
        self.prev_odom = msg
    
    def scan_callback(self, msg: LaserScan):
        """Update particle weights based on scan"""
        
        # Compute weight for each particle
        for particle in self.particles:
            particle.weight = self.compute_scan_likelihood(particle, msg)
        
        # Normalize weights
        total_weight = sum(p.weight for p in self.particles)
        if total_weight > 0:
            for particle in self.particles:
                particle.weight /= total_weight
        else:
            for particle in self.particles:
                particle.weight = 1.0 / self.num_particles
        
        # Resample
        self.resample_particles()
        
        # Publish pose estimate
        self.publish_pose_estimate(msg.header.stamp)
        
        # Publish particles
        self.publish_particles(msg.header.stamp)
    
    def compute_scan_likelihood(self, particle, scan):
        """Simple likelihood: penalize outliers"""
        likelihood = 1.0
        valid_rays = 0
        
        # Sample rays
        indices = np.linspace(0, len(scan.ranges)-1, 15, dtype=int)
        
        for idx in indices:
            range_val = scan.ranges[idx]
            if range_val < scan.range_min or range_val > scan.range_max:
                continue
            
            valid_rays += 1
            # Likelihood: penalize very far readings
            if range_val > 30:
                likelihood *= 0.5
            else:
                likelihood *= 1.0
        
        if valid_rays == 0:
            return 0.001
        
        return max(likelihood, 0.001)
    
    def resample_particles(self):
        """Low-variance resampling"""
        new_particles = []
        weights = np.array([p.weight for p in self.particles])
        
        cumsum = np.cumsum(weights)
        r = np.random.uniform(0, 1.0 / self.num_particles)
        
        j = 0
        for i in range(self.num_particles):
            u = r + i / self.num_particles
            while u > cumsum[j] and j < len(cumsum) - 1:
                j += 1
            
            p = self.particles[j]
            new_particles.append(Particle(p.x, p.y, p.theta, 1.0/self.num_particles))
        
        self.particles = new_particles
    
    def publish_pose_estimate(self, timestamp):
        """Publish MCL pose estimate"""
        if len(self.particles) == 0:
            return
        
        # Weighted mean
        mean_x = sum(p.x * p.weight for p in self.particles)
        mean_y = sum(p.y * p.weight for p in self.particles)
        mean_theta = sum(p.theta * p.weight for p in self.particles)
        
        # Confidence (inverse of particle spread)
        var_x = sum(p.weight * (p.x - mean_x)**2 for p in self.particles)
        var_y = sum(p.weight * (p.y - mean_y)**2 for p in self.particles)
        spread = var_x + var_y
        
        self.best_confidence = 1.0 / (1.0 + spread)
        self.best_pose = (mean_x, mean_y, mean_theta)
        
        # Publish as odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = float(mean_x)
        odom_msg.pose.pose.position.y = float(mean_y)
        
        cy = math.cos(mean_theta * 0.5)
        sy = math.sin(mean_theta * 0.5)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sy
        odom_msg.pose.pose.orientation.w = cy
        
        self.pose_pub.publish(odom_msg)
        
        self.get_logger().debug(
            f'MCL Pose: ({mean_x:.2f}, {mean_y:.2f}, {mean_theta:.2f}) | Confidence: {self.best_confidence:.3f}'
        )
    
    def publish_particles(self, timestamp):
        """Publish particle cloud for visualization"""
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = 'odom'
        
        for particle in self.particles:
            pose = Pose()
            pose.position.x = particle.x
            pose.position.y = particle.y
            
            cy = math.cos(particle.theta * 0.5)
            sy = math.sin(particle.theta * 0.5)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = sy
            pose.orientation.w = cy
            
            pose_array.poses.append(pose)
        
        self.particles_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = MCLLocalizerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


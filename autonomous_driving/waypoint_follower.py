#!/usr/bin/env python3
import rclpy, json, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import Quaternion

def quat_to_yaw(q: Quaternion):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))

class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')
        
        # PARAMETERS (tunable from command line)
        self.declare_parameter('throttle', 0.08)
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('wheelbase', 0.4)
        
        self.throttle = self.get_parameter('throttle').value
        self.lookahead = self.get_parameter('lookahead').value
        self.wheelbase = self.get_parameter('wheelbase').value
        
        self.waypoints = json.load(open('/workspace/ros2_ws/waypoints.json'))
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.odom_sub = self.create_subscription(Odometry, '/testing_only/odom', self.odom_cb, 10)
        self.pos = None
        self.yaw = 0.0
        self.idx = 0
        self.create_timer(0.05, self.control_loop)  # 20Hz
        self.get_logger().info(f'✅ Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info(f'🚗 Params: throttle={self.throttle}, lookahead={self.lookahead}, wheelbase={self.wheelbase}')

    def odom_cb(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

    def control_loop(self):
        if self.pos is None or not self.waypoints: return
        
        # Find closest waypoint
        closest_dist = float('inf')
        for i, wp in enumerate(self.waypoints):
            d = math.hypot(wp[0]-self.pos[0], wp[1]-self.pos[1])
            if d < closest_dist:
                closest_dist = d
                self.idx = i

        # Find lookahead point
        target = None
        for i in range(1, len(self.waypoints)):
            idx = (self.idx + i) % len(self.waypoints)
            wp = self.waypoints[idx]
            d = math.hypot(wp[0]-self.pos[0], wp[1]-self.pos[1])
            if d >= self.lookahead:
                target = wp
                break
        
        if target is None:
            target = self.waypoints[(self.idx+1) % len(self.waypoints)]

        # Pure pursuit
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        d = math.hypot(dx, dy)
        if d < 0.01:
            steer = 0.0
        else:
            alpha = math.atan2(dy, dx)
            error = alpha - self.yaw
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            steer = math.atan2(2 * self.wheelbase * math.sin(error), d)
            steer = max(-1.0, min(1.0, steer))

        msg = ControlCommand()
        msg.throttle = self.throttle
        msg.steering = float(steer)
        msg.brake = 0.0
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SimpleFollower())
    rclpy.shutdown()

if __name__ == '__main__':
    main()


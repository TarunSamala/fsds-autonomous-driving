import rclpy
from rclpy.node import Node
import time

from fs_msgs.msg import ControlCommand


class ControlTest(Node):
    def __init__(self):
        super().__init__('control_test')

        self.publisher = self.create_publisher(
            ControlCommand,
            '/fsds/control_command',
            10)

        self.get_logger().info('Control Test Node Started')
        self.get_logger().info('Will send: kick throttle → wait 2s → reset to zero')

    def run_test(self):
        """Send a test sequence."""
        
        # Wait for subscribers to connect
        time.sleep(1)

        # 1. Send throttle kick (0.3)
        self.get_logger().info('>>> Sending THROTTLE KICK (0.6)')
        cmd = ControlCommand()
        cmd.throttle = 0.6
        cmd.steering = 0.0
        cmd.brake = 0.0
        
        for i in range(10):  # Publish for 1 second (10 × 0.1s)
            self.publisher.publish(cmd)
            time.sleep(0.1)

        # 2. Wait 2 seconds (car coasting)
        self.get_logger().info('>>> Coasting for 2 seconds...')
        time.sleep(2)

        # 3. Reset to zero
        self.get_logger().info('>>> Resetting to ZERO')
        cmd = ControlCommand()
        cmd.throttle = 0.0
        cmd.steering = 0.0
        cmd.brake = 0.0
        
        for i in range(10):  # Publish for 1 second
            self.publisher.publish(cmd)
            time.sleep(0.1)

        self.get_logger().info('>>> Test complete!')


def main(args=None):
    rclpy.init(args=args)
    node = ControlTest()
    node.run_test()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


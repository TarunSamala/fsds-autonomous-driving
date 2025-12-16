import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class SimpleImuListener(Node):
    def __init__(self):
        super().__init__('simple_imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.callback,
            10)
        self.get_logger().info('Simple IMU listener started')

    def callback(self, msg: Imu):
        self.get_logger().info(f'IMU z-accel: {msg.linear_acceleration.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleImuListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


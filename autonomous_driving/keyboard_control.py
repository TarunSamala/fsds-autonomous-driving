import rclpy
from rclpy.node import Node
import sys
import termios
import tty
import threading
import time

from fs_msgs.msg import ControlCommand


def getch():
    """Read one character from stdin (blocking, raw mode)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            10)

        # State
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0

        # Parameters
        self.throttle_step = 0.02    # throttle increment per keypress
        self.steering_increment = 0.25  # per keypress (0.25, 0.5, 0.75, 1.0)
        self.brake_step = 0.3
        self.kickstart_value = 0.2
        self.kickstart_duration = 0.5  # 0.5 seconds to overcome friction

        self.get_logger().info('Keyboard Controller Started (terminal-based)')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W: Accelerate (+0.02 per press, fine control)')
        self.get_logger().info('  S: Brake (+0.3, throttle stops)')
        self.get_logger().info('  A: Steer Left (-0.25, -0.5, -0.75, -1.0) - resets when switching from D')
        self.get_logger().info('  D: Steer Right (+0.25, +0.5, +0.75, +1.0) - resets when switching from A')
        self.get_logger().info('  SPACE: Reset (all zero)')
        self.get_logger().info('  Q: Quit')

        self.get_logger().info('')
        self.get_logger().info('🚀 KICKSTART INITIATED!')
        self.get_logger().info(f'  Sending throttle = 0.2 for 0.5 seconds to overcome friction...')
        self.get_logger().info('')

        # Flags / threads
        self.running = True

        # Kickstart thread
        self.kick_thread = threading.Thread(
            target=self.kickstart_sequence, daemon=True)
        self.kick_thread.start()

        # Periodic publisher (manual control values)
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # Keyboard input thread
        self.kb_thread = threading.Thread(
            target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

    def kickstart_sequence(self):
        """Send throttle boost to overcome friction, then STOP the car completely."""
        # 0.5 seconds of throttle 0.2 to get the car moving
        num_ticks = int(self.kickstart_duration / 0.1)  # 5 ticks for 0.5s
        for _ in range(num_ticks):
            msg = ControlCommand()
            msg.throttle = self.kickstart_value  # 0.2
            msg.steering = 0.0
            msg.brake = 0.0
            self.pub.publish(msg)
            time.sleep(0.1)

        # IMMEDIATELY stop: set throttle to 0, apply brake
        self.get_logger().info('✅ KICKSTART PULSE SENT!')
        self.get_logger().info('  Stopping car now...')
        
        # Send brake command for 1 second to ensure car stops
        for _ in range(10):  # 10 * 0.1s = 1s
            msg = ControlCommand()
            msg.throttle = 0.0
            msg.steering = 0.0
            msg.brake = 0.5  # Apply brake to stop
            self.pub.publish(msg)
            time.sleep(0.1)

        # Reset to zero for manual control
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0

        self.get_logger().info('✅ CAR STOPPED!')
        self.get_logger().info('  Ready for manual control (WASD)')
        self.get_logger().info('')

    def keyboard_loop(self):
        """Capture keyboard input in a loop."""
        while self.running and rclpy.ok():
            ch = getch()
            if not ch:
                continue

            c = ch.lower()

            if c == 'w':
                # Throttle: increment by 0.02 each press
                self.throttle = min(1.0, self.throttle + self.throttle_step)
                self.brake = 0.0
                self.get_logger().info(f'Throttle: {self.throttle:.2f}')

            elif c == 's':
                # Brake
                self.brake = min(1.0, self.brake + self.brake_step)
                self.throttle = 0.0  # Stop throttle when braking
                self.get_logger().info(f'Brake: {self.brake:.2f}')

            elif c == 'a':
                # Steer left: accumulate negative values
                # If was positive (right), reset to 0 first
                if self.steering > 0:
                    self.steering = 0.0
                    self.get_logger().info(f'Steering: {self.steering:.2f} (CENTER - reset)')
                else:
                    # Accumulate left: -0.25, -0.5, -0.75, -1.0
                    self.steering = max(-1.0, self.steering - self.steering_increment)
                    self.get_logger().info(f'Steering: {self.steering:.2f} (LEFT)')

            elif c == 'd':
                # Steer right: accumulate positive values
                # If was negative (left), reset to 0 first
                if self.steering < 0:
                    self.steering = 0.0
                    self.get_logger().info(f'Steering: {self.steering:.2f} (CENTER - reset)')
                else:
                    # Accumulate right: 0.25, 0.5, 0.75, 1.0
                    self.steering = min(1.0, self.steering + self.steering_increment)
                    self.get_logger().info(f'Steering: {self.steering:.2f} (RIGHT)')

            elif ch == ' ':
                self.throttle = 0.0
                self.brake = 0.0
                self.steering = 0.0
                self.get_logger().info('Reset: throttle=0, brake=0, steering=0')

            elif c == 'q':
                self.get_logger().info('Quit requested')
                self.running = False
                rclpy.shutdown()
                break

    def publish_cmd(self):
        """Publish the current manual control command at 10 Hz."""
        msg = ControlCommand()
        msg.throttle = float(self.throttle)
        msg.steering = float(self.steering)
        msg.brake = float(self.brake)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.running = False
        rclpy.shutdown()


if __name__ == '__main__':
    main()


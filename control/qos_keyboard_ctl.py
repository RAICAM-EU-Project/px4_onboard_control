import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Define custom QoS for better real-time control
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Publisher with custom QoS
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Motion parameters
        self.linear_speed = 0.0       # current linear speed
        self.angular_speed = 0.0      # current angular speed
        self.accel_step = 0.02        # increment per key press (m/s)
        self.turn_step = 0.2          # increment per key press (rad/s)
        self.max_linear = 1.2         # max linear speed
        self.max_angular = 1.0        # max angular speed

        self._running = True

        self.get_logger().info(
            "Keyboard Teleop started.\n"
            "W/S: accelerate forward/backward\n"
            "A/D: turn right/left (steering direction reversed)\n"
            "X: stop immediately\n"
            "CTRL+C: quit"
        )

        # Thread for reading the keyboard
        kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        kb_thread.start()

        # Thread for continuously publishing the current Twist
        pub_thread = threading.Thread(target=self.publish_loop, daemon=True)
        pub_thread.start()

    def keyboard_loop(self):
        """Read one character at a time and adjust speeds."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)

        try:
            while self._running:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'w':
                        self.linear_speed = min(self.linear_speed + self.accel_step,
                                                self.max_linear)
                    elif key == 's':
                        self.linear_speed = max(self.linear_speed - self.accel_step,
                                                -self.max_linear)
                    elif key == 'a':
                        self.angular_speed = max(self.angular_speed - self.turn_step,
                                                 -self.max_angular)
                    elif key == 'd':
                        self.angular_speed = min(self.angular_speed + self.turn_step,
                                                 self.max_angular)
                    elif key == 'x':
                        self.linear_speed = 0.0
                        self.angular_speed = 0.0
                    elif key == '\x03':  # Ctrl+C
                        self._running = False
                        break
                    else:
                        continue

                    self.get_logger().info(
                        f"Updated speeds → linear: {self.linear_speed:.2f} m/s, "
                        f"angular: {self.angular_speed:.2f} rad/s"
                    )
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def publish_loop(self):
        """Publish the current Twist at 10 Hz."""
        twist = Twist()
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok() and self._running:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.pub.publish(twist)
            rate.sleep()

    def destroy(self):
        self._running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Teleop stopped by user")
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

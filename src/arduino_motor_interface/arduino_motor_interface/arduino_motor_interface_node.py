import rclpy
from rclpy.node import Node
import serial
import time

from geometry_msgs.msg import Twist

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('arduino_motor_interface')

        # Replace with your serial device
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Listening to /cmd_vel and sending motor commands to Arduino")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Simple tank drive conversion
        left_speed = linear - angular
        right_speed = linear + angular

        # Clamp and scale
        def scale(val):
            return int(max(min(val, 1.0), -1.0) * 100)

        lspd = scale(left_speed)
        rspd = scale(right_speed)

        # Build and send motor commands
        commands = [
            f"lefttrack:{'F' if lspd > 0 else 'B' if lspd < 0 else 'R'}:{abs(lspd)}",
            f"righttrack:{'F' if rspd > 0 else 'B' if rspd < 0 else 'R'}:{abs(rspd)}"
        ]

        for cmd in commands:
            self.serial_port.write((cmd + '\n').encode())
            self.get_logger().info(f"Sent: {cmd}")

            time.sleep(0.1)

            line = self.serial_port.readline().decode().strip()
            if line:
                self.get_logger().info(f"[Arduino] {line}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


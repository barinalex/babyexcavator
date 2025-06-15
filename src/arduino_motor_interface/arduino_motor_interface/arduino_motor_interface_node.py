import rclpy
from rclpy.node import Node
import serial
import time

from std_msgs.msg import String

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('arduino_motor_interface')

        # Replace with your serial device
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        self.subscription = self.create_subscription(
            String,
            '/arduino/motor_control',
            self.motor_command_callback,
            10
        )

        self.get_logger().info("Listening to /arduino/motor_control and sending motor commands to Arduino")

    def motor_command_callback(self, msg):
        cmd = msg.data.strip()

        try:
            self.serial_port.write((cmd + '\n').encode())
            self.get_logger().info(f"Sent to Arduino: {cmd}")

            time.sleep(0.05)

            response = self.serial_port.readline().decode().strip()
            if response:
                self.get_logger().info(f"[Arduino] {response}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial write/read failed: {e}")

    def close(self):
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Closed serial port")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


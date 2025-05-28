import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import re

class SerialADCReader(Node):
    def __init__(self):
        super().__init__('serial_adc_reader')
        self.publisher_ = self.create_publisher(Int32, '/adc_values', 10)
        self.serial_port = serial.Serial('/dev/pts/8', 9600, timeout=1)
        self.timer = self.create_timer(0.2, self.read_serial)
        self.pattern = re.compile(r"!E(\d{4})D")

    def read_serial(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        match = self.pattern.fullmatch(line)
        if match:
            adc_value = int(match.group(1))
            msg = Int32()
            msg.data = adc_value
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published ADC value: {adc_value}")
        else:
            self.get_logger().warn(f"Ignored invalid line: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialADCReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

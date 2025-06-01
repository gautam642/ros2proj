# motor_sim.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MotorSimulator(Node):
    def __init__(self):
        super().__init__('motor_sim')
        self.pwm = 0
        self.rpm = 0.0
        self.publisher = self.create_publisher(Int32, 'adc_values', 10)
        self.subscription = self.create_subscription(Int32, 'motor_pwm', self.pwm_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_rpm)

    def pwm_callback(self, msg):
        self.pwm = msg.data
        self.get_logger().info(f"Received PWM: {self.pwm}")

    def publish_rpm(self):
        # Simulate motor response to PWM (simple first-order lag model)
        self.rpm += 0.3 * ((self.pwm / 255.0) * 3000 - self.rpm) # Scale: PWM 255 ≈ 1500 RPM
        rpm_int = int(self.rpm)
        msg = Int32()
        msg.data = rpm_int
        self.publisher.publish(msg)
        self.get_logger().info(f"Simulated RPM: {rpm_int}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

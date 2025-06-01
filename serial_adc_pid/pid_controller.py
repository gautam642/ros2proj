import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(
            Int32,
            '/adc_values',
            self.listener_callback,
            10
        )

        self.target_rpm = 1500  #(0-3000)
        self.kp = 0.5
        self.ki = 0.05
        self.kd = 0.1

        self.last_error = 0
        self.integral = 0

        self.motor_command_pub = self.create_publisher(Int32, '/motor_pwm', 10)

    def listener_callback(self, msg):
        current_rpm = msg.data  # Assuming ADC value maps directly to RPM
        error = self.target_rpm - current_rpm
        self.integral += error
        derivative = error - self.last_error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        pwm_signal = int(min(max((output / 3000.0) * 255, 0), 255))   # Clamp to 0-255 for PWM

        pwm_msg = Int32()
        pwm_msg.data = pwm_signal
        self.motor_command_pub.publish(pwm_msg)

        self.get_logger().info(
            f"Target: {self.target_rpm}, RPM: {current_rpm}, PWM: {pwm_signal}"
        )

        self.last_error = error

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

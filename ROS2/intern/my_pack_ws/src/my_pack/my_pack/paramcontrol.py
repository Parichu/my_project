#/usr/bin/env python3
import rclpy
from rclpy.node import  Node 
from std_msgs.msg import Int32
class MotorControlPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_pub')
        self.publisher_ = self.create_publisher(Int32, 'intern_send_pwm', 10)
        self.declare_parameter('pwm', 0)
        self.pwm = self.get_parameter('pwm').value
        self.create_timer(0.005, self.talker_callback)
    def talker_callback(self):
        self.pwm = self.get_parameter('pwm').get_parameter_value().integer_value
        msg = Int32()
        msg.data = self.pwm
        self.publisher_.publish(msg)
        self.get_logger().info(f'Duty cycle: {(self.pwm/255*100):.2f} %')
def main(args=None):
    rclpy.init(args=args)
    node = MotorControlPublisher()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
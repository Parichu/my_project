#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import math

class Motor(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=10)  # แก้เป็น ACM0 ถ้าจำเป็น
            self.get_logger().info('Serial port opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            exit(1)

        self.mm_per_pulse = (math.pi * 65) / 616


    def listener_callback(self, msg):
        wheel_base = 0.15

        v = msg.linear.x
        omega = msg.angular.z

        v_l = v - (wheel_base / 2.0) * omega
        v_r = v + (wheel_base / 2.0) * omega

        left_pulse = int((v_l * 1000) / self.mm_per_pulse) if v_l != 0 else 0
        right_pulse = int((v_r * 1000) / self.mm_per_pulse) if v_r != 0 else 0

        
        try:
            header = b'\x00'
            packet = header + struct.pack('<ii', left_pulse, right_pulse)
            self.ser.write(packet)
            self.get_logger().info(f"Sent target pulse: L={left_pulse}, R={right_pulse}")
        except Exception as e:
            self.get_logger().error(f"Failed to send PWM: {e}")

            

def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    
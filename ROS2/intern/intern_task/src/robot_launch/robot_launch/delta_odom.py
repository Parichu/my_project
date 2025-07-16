#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
import serial
import struct
import math
import time

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        port_name = self.get_parameter('port').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(port_name, 9600, timeout=1)
            self.get_logger().info(f"Serial opened: {port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            exit(1)

        # Robot parameters
        self.wheel_radius = 0.065 / 2     # meters
        self.wheel_base = 0.15            # meters
        self.ppr = 616                    # Pulse Per Revolution
        self.mm_per_pulse = (math.pi * 65) / self.ppr  # mm/pulse

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        raw = self.ser.read(9)
        if len(raw) != 9 or raw[0] != 0x00:
            return

        try:
            delta_left, delta_right = struct.unpack('<ii', raw[1:])
        except struct.error as e:
            self.get_logger().error(f"Struct unpack error: {e}")
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        dt = now_sec - self.prev_time
        self.prev_time = now_sec

        d_left = delta_left * self.mm_per_pulse / 1000.0  # m
        d_right = delta_right * self.mm_per_pulse / 1000.0  # m
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # Update pose
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Build Odometry msg
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion()
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt

        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(f"L: {delta_left}, R: {delta_right}")

        # self.get_logger().info(f"L: {delta_left} {d_left} | R: {delta_right} {d_right}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    
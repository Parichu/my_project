#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar
import math

class RPLidarNode(Node):
    def __init__(self):
        super().__init__('rplidar_node')

        self.lidar_port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.lidar = RPLidar(self.lidar_port)

        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)

        # ตั้งค่าพารามิเตอร์ LaserScan
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = 'laser_frame'
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = 2 * math.pi
        self.scan_msg.angle_increment = math.radians(1.0)  # 1 degree
        self.scan_msg.time_increment = 0.0
        self.scan_msg.scan_time = 0.1
        self.scan_msg.range_min = 0.15
        self.scan_msg.range_max = 6.0

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.scan_data = [float('inf')] * 360  # เก็บระยะตามมุม

    def timer_callback(self):
        # ดึงข้อมูลสแกน
        try:
            scans = next(self.lidar.iter_scans())
            # reset
            self.scan_data = [float('inf')] * 360
            for (_, angle, distance) in scans:
                angle_deg = int(angle)
                if 0 <= angle_deg < 360:
                    self.scan_data[angle_deg] = distance / 1000.0  # mm to meters

            # กรอกข้อมูล LaserScan message
            self.scan_msg.header.stamp = self.get_clock().now().to_msg()
            self.scan_msg.ranges = self.scan_data

            self.publisher_.publish(self.scan_msg)
            self.get_logger().info('Published scan')
        except Exception as e:
            self.get_logger().error(f'Error reading scan: {e}')

    def destroy_node(self):
        self.lidar.stop()
        self.lidar.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

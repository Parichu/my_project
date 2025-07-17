#!/usr/bin/env python3
import rclpy
from task_service.srv import Calculate
import rclpy
from rclpy.node import Node

class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.cli = self.create_client(Calculate, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for add service...')
        self.req = Calculate.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)
    
def main(args=None):
    rclpy.init(args=args)
    run = AddClient()
    rclpy.spin(run)
    run.destroy_node()
    rclpy.shutdown()

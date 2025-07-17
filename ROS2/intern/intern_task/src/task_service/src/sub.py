#! /usr/bin/env python3

import rclpy

from rclpy.node import Node
from task_service.srv import Calculate

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(Calculate, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = Calculate.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

rclpy.init()
client = AddTwoIntsClient()
response = client.send_request(3, 7)
print(f'Result: {response.sum}')
rclpy.shutdown()
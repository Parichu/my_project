#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from task_service.srv import Calculate

class CalculateService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(Calculate, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Received: a={request.a}, b={request.b}')
        return response

rclpy.init()
node = CalculateService()
rclpy.spin(node)
rclpy.shutdown()

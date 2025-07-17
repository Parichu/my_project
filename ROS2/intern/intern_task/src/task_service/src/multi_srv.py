#!/usr/bin/env python3
import rclpy
from task_service.srv import Calculate
from task_service.srv import MultiplyTwoInts  
from rclpy.node import Node

class MultiServiceServer(Node):
    def __init__(self):
        super().__init__('multi_service_server')

        self.srv_add = self.create_service(Calculate, 'add_two_ints', self.add_callback)
        self.srv_mul = self.create_service(MultiplyTwoInts, 'multiply_two_ints', self.mul_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Add: {request.a} + {request.b} = {response.sum}')
        return response

    def mul_callback(self, request, response):
        response.product = request.a * request.b
        self.get_logger().info(f'Mul: {request.a} * {request.b} = {response.product}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MultiServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

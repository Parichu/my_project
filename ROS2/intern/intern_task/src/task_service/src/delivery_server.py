#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import GoalResponse, CancelResponse
from task_service.srv import DeliveryRequest
from task_service.action import DeliveryItems

class ServiceAction(Node):
    def __init__(self):
        super().__init__('delivery_service')

        # 🔧 สร้าง ROS2 Service
        self.srv = self.create_service(
            DeliveryRequest,
            'request_room',
            self.request_callback
        )

        # 🚚 สร้าง Action Server
        self._action_server = ActionServer(
            self,
            DeliveryItems,
            'delivery_item',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,           # optional
            cancel_callback=self.cancel_callback         # optional
        )

    def request_callback(self, request, response):
        self.get_logger().info(f'📩 Receive request: {request.room_name}')
        valid_rooms = ['A01', 'A02', 'A03', 'B01', 'B02', 'B03']

        if request.room_name.upper() in valid_rooms:
            self.get_logger().info("✅ Room is valid. Accepting request.")
            response.accepted = True
            response.message = f"Now moving to... {request.room_name}"
            self.room_name = request.room_name
        else:
            self.get_logger().warn("❌ Room not found.")
            response.accepted = False
            response.message = f"Not found room: {request.room_name}"

        return response

    def goal_callback(self, goal_request):
        self.get_logger().info(f"🎯 Received goal request: {goal_request}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info(f"❌ Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'🚀 Starting delivery to: {goal_handle.request.room_name}')

        feedback_msg = DeliveryItems.Feedback()
        result = DeliveryItems.Result()

        for i in range(1, 11):
            feedback_msg.current_location = f"Moving... {i * 10}%"
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"📦 Feedback: {feedback_msg.current_location}")
            time.sleep(1)

        self.get_logger().info("✅ Delivery completed")
        result.final_message = f"Delivered to {goal_handle.request.room_name} successfully."
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ServiceAction()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

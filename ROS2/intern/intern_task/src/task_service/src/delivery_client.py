#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from task_service.srv import DeliveryRequest
from task_service.action import DeliveryItems


class DeliveryClient(Node):
    def __init__(self):
        super().__init__('delivery_client')

        # สร้าง Service Client
        self.cli = self.create_client(DeliveryRequest, 'request_room')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('🕒 Waiting for service server...')

        self.req = DeliveryRequest.Request()

        # สร้าง Action Client
        self._action_client = ActionClient(self, DeliveryItems, 'delivery_item')

    def send_request(self, room_name):
        self.req.room_name = room_name
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.accepted:
            print(f"✅ Service approved. Message: {response.message}")
            self.send_goal(room_name)
        else:
            print(f"❌ Service rejected. Message: {response.message}")
            rclpy.shutdown()

    def send_goal(self, room_name):
        while not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('🕒 Waiting for action server...')

        goal_msg = DeliveryItems.Goal()
        goal_msg.room_name = room_name

        print(f"🚀 Sending goal to deliver to {room_name}...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"📦 Feedback: {feedback.current_location}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('❌ Goal rejected by ActionServer.')
            rclpy.shutdown()
            return

        print('✅ Goal accepted by ActionServer.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(f"🎉 Action result: {result.final_message}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = DeliveryClient()

    room_name = input("🏠 Enter Room Name: ").strip().upper()
    client.send_request(room_name)

    rclpy.spin(client)


if __name__ == '__main__':
    main()

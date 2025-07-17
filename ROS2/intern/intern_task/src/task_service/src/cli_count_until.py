#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from task_service.action import CountUntil

class CountUntilClient(Node):
    def __init__(self):
        super().__init__('count_until_client')
        self._client = ActionClient(self, CountUntil, 'count_until')

    def send_goal(self, target_number):
        self._client.wait_for_server()
        goal_msg = CountUntil.Goal()
        goal_msg.target_number = target_number

        self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: current_number = {feedback.current_number}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success = {result.success}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient()
    node.send_goal(5)  # เปลี่ยนเป็นเลขที่อยากนับถึง
    rclpy.spin(node)

if __name__ == '__main__':
    main()

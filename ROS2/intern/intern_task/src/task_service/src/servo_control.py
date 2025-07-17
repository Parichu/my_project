#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from task_service.action import Feedbackservo


class ServoClient(Node):
    def __init__(self):
        super().__init__('servo_client')
        self._action_client = ActionClient(self, Feedbackservo, 'move_servo')

    def send_goal(self, angle):
        goal_msg = Feedbackservo.Goal()
        goal_msg.target_angle = angle

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'[Feedback] Current Angle: {feedback_msg.feedback.current_angle}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected 😢')
            return

        self.get_logger().info('Goal accepted! ✅')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Final angle: {result.final_angle}')

def main(args=None):
    rclpy.init(args=args)
    client = ServoClient()
    client.send_goal(90)  # ตั้งเป้าไปที่ 90 องศา
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
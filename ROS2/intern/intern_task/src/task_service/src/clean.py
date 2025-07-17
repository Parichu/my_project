#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from task_service.action import ClassRoom

class CleanRoomClient(Node):
    def __init__(self):
        super().__init__('clean_room_client')
        self._client = ActionClient(self, ClassRoom, 'clean_room')

    def send_goal(self, width, length, worker = 1):
        self._client.wait_for_server()
        goal_msg = ClassRoom.Goal()
        goal_msg.width = width
        goal_msg.length = length
        goal_msg.worker = worker

        self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.progress_percentage:.1f}%')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result_message} (Time: {result.total_time:.2f} s)')


def main(args=None):
    rclpy.init(args=args)
    node = CleanRoomClient()
    node.send_goal(width=25.0, length=45.0)  # พื้นที่ 25x45 เมตร
    rclpy.spin(node)


if __name__ == '__main__':
    main()

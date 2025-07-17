#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from task_service.action import CountUntil
import time

class CountUntilActionServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: {goal_handle.request.target_number}')

        current_number = 0
        target = goal_handle.request.target_number

        while current_number <= target:
            feedback = CountUntil.Feedback()
            feedback.current_number = current_number
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Publishing feedback: {current_number}')

            time.sleep(1)
            current_number += 1

        goal_handle.succeed()
        result = CountUntil.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

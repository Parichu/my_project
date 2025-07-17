#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from task_service.action import ClassRoom

import time

class CleanRoomServer(Node):
    def __init__(self):
        super().__init__('clean_room_server')
        self._action_server = ActionServer(
            self,
            ClassRoom,
            'clean_room',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback)

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        width = goal_handle.request.width
        length = goal_handle.request.length
        worker = max(goal_handle.request.worker, 1)
        area = width * length

        self.get_logger().info(f'เริ่มเช็ดพื้นที่ขนาด {width} x {length} ตร.เมตร')

        feedback_msg = ClassRoom.Feedback()
        progress = 0.0

        total_steps = 100
        start_time = time.time()

        speed_facot = 1.0 / worker

        for i in range(total_steps):
            if goal_handle.is_cancel_requested:
                elapsed = time.time() - start_time
                goal_handle.canceled()
                result = ClassRoom.Result()
                result.total_time = float(elapsed)
                result.result_message = f'ยกเลิกภารกิจตอน {progress:.1f}%'
                self.get_logger().info(result.result_message)
                return result

            time.sleep(0.1 * area / 10)  # ยิ่งพื้นที่มากยิ่งช้า
            progress = (i + 1) / total_steps * 100.0
            feedback_msg.progress_percentage = progress
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'กำลังเช็ด... {progress:.1f}%')

        elapsed = time.time() - start_time
        goal_handle.succeed()

        result = ClassRoom.Result()
        result.total_time = float(elapsed)
        result.result_message = 'ทำความสะอาดเสร็จเรียบร้อย'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CleanRoomServer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

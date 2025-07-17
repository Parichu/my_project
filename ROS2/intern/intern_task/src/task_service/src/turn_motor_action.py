#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from task_service.action import TurnMotor
import serial
import time

class MotorControlServer(Node):
    def __init__(self):
        super().__init__('motor_action_server')
        try:
            self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=2)
            time.sleep(2)  # ให้ Arduino reset
            self.get_logger().info("✅ เชื่อมต่อ Serial แล้ว: /dev/ttyACM1")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ ไม่สามารถเชื่อมต่อ: {e}")
            self.ser = None

        self._action_server = ActionServer(
            self, TurnMotor, 'turn_motor', self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        direction = goal_handle.request.direction
        speed = goal_handle.request.speed
        cmd = f"DIR:{direction} SPD:{speed}\n"

        if self.ser and self.ser.is_open:
            self.ser.write(cmd.encode())
            self.ser.flush()
            self.get_logger().info(f"📤 ส่งคำสั่งไป Arduino: {cmd.strip()}")

            # รอ feedback จาก Arduino
            try:
                response = self.ser.readline().decode().strip()
                self.get_logger().info(f"📩 Arduino ตอบกลับ: {response}")
            except Exception as e:
                self.get_logger().error(f"⚠️ อ่าน Serial ไม่ได้: {e}")

            feedback_msg = TurnMotor.Feedback()
            feedback_msg.feedback = f"กำลังควบคุมมอเตอร์: {cmd.strip()}"
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)

            goal_handle.succeed()
            result = TurnMotor.Result()
            result.success = True
            return result
        else:
            self.get_logger().error("❌ Serial ยังไม่เปิด")
            goal_handle.abort()
            result = TurnMotor.Result()
            result.success = False
            return result

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from task_service.action import ControlServo  # Action ชื่อ ControlServo
import serial

class PIRListenerNode(Node):
    def __init__(self):
        super().__init__('pir_listener')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.create_timer(0.1, self.read_serial)
        self._action_client = ActionClient(self, ControlServo, 'servo_control')
        self.last_state = None

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if line.startswith("PIR:"):
                state = line.split(":")[1]
                if state != self.last_state:
                    self.send_goal(int(state))
                    self.last_state = state

    def send_goal(self, val):
        from task_service.action import ControlServo
        goal_msg = ControlServo.Goal()
        goal_msg.target_angle = 90 if val == 1 else 0
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIRListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
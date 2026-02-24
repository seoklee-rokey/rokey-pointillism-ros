#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from dot_msgs.action import DrawStipple
from dot_msgs.msg import Dot, DotArray


class DrawStippleDummyClient(Node):
    def __init__(self):
        super().__init__('draw_stipple_dummy_client')
        self.client = ActionClient(self, DrawStipple, '/draw_stipple')

    def send_dummy_goal(self):
        # 1) DotArray 만들기 (x,y는 "정규화 0~1" 가정)
        arr = DotArray()

        for x, y, v in [
            (450, -30, 1),
            (450, -25, 1),
            (450, -10, 2),
            (450, -5, 2),
            (450, 10, 3),
            (450, 15, 3),
            (450, 30, 4),
            (450, 35, 4),
            (450, 50, 5),
            (450, 55, 5),
            (450, 70, 6),
            (450, 75, 6),

            (400, -30, 7),
            (400, -25, 7),
            (400, -10, 8),
            (400, -5, 8),
            (400, 10, 9),
            (400, 15, 9),
            (400, 30, 10),
            (400, 35, 10),
            (400, 50, 11),
            (400, 55, 11),
            (400, 70, 12),
            (400, 75, 12),

            (300, -30, 13),
            (300, -25, 13),
            (300, -10, 14),
            (300, -5, 14),
            (300, 10, 15),
            (300, 15, 15),
            (300, 30, 16),
            (300, 35, 16),
            (300, 50, 17),
            (300, 55, 17),
            (300, 70, 18),
            (300, 75, 18),
        ]:
            d = Dot()
            d.x = float(x)
            d.y = float(y)
            d.v = int(v)  # uint8
            arr.dots.append(d)

        goal = DrawStipple.Goal()
        goal.data = arr

        # 2) 서버 대기 후 전송
        self.get_logger().info("Waiting for /draw_stipple action server...")
        self.client.wait_for_server()

        self.get_logger().info(f"Sending goal with {len(arr.dots)} dots...")
        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(f"feedback: {fb.percent:.1f}% (current_v={fb.current_v})")

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        res = future.result().result
        self.get_logger().info(f"Result: success={res.success}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = DrawStippleDummyClient()
    node.send_dummy_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

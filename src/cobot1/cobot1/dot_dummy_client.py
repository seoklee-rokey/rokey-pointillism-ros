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
            (0.20, 0.20, 1),
            (0.40, 0.20, 1),
            (0.40, 0.40, 1),
            (0.20, 0.40, 1),
            (0.25, 0.20, 2),
            (0.45, 0.20, 2),
            (0.45, 0.40, 2),
            (0.25, 0.40, 2),
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

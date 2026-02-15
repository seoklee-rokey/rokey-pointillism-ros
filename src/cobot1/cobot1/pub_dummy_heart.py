#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from dot_msgs.msg import Dot, DotArray

class DummyHeartOutlinePublisher(Node):
    def __init__(self):
        super().__init__("dummy_heart_outline_publisher")
        self.pub = self.create_publisher(DotArray, "/draw_this", 10)

        # 한 번만 발행
        self.timer = self.create_timer(0.5, self.publish_once)
        self.sent = False

    def publish_once(self):
        if self.sent:
            return
        self.sent = True

        # ---- 파라미터 ----
        num_points = 20      # 점 개수(늘릴수록 매끄러움)
        center_x = 300.0
        center_y = 50.0
        scale = 5.0           # 하트 크기
        v_fixed = 1          # 1~18 (진하기/강도)
        # ---------------

        msg = DotArray()
        msg.dots = []

        # 하트 외곽선(표준 parametric)
        # x = 16 sin^3(t)
        # y = 13 cos(t) - 5 cos(2t) - 2 cos(3t) - cos(4t)
        for k in range(num_points):
            t = 2.0 * math.pi * k / num_points

            x_raw = 16.0 * (math.sin(t) ** 3)
            y_raw = (13.0 * math.cos(t)
                     - 5.0 * math.cos(2.0 * t)
                     - 2.0 * math.cos(3.0 * t)
                     - 1.0 * math.cos(4.0 * t))

            d = Dot()
            d.x = float(center_x + scale * x_raw)
            d.y = float(center_y + scale * y_raw)
            d.v = int(v_fixed)     # ✅ 1~18

            msg.dots.append(d)

        self.pub.publish(msg)
        self.get_logger().info(f"/draw_this로 하트 외곽선 {len(msg.dots)}점 publish (v={v_fixed})")

def main():
    rclpy.init()
    node = DummyHeartOutlinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

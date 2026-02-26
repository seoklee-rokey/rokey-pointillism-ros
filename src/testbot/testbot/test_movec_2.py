#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 100
ACC = 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(0.5)


def perform_task():
    from DSR_ROBOT2 import posx, movesx, movej

    # 준비자세(옵션) - 필요없으면 지워도 됨
    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)

    # ===== 테스트 파라미터 =====
    N_DOTS = 20

    # 시작점/도착점 (XY만 선형 보간)
    x0, y0 = 200.0, 0.0
    x1, y1 = 450.0, 150.0

    # 자세 (r,p,y) 고정
    rx, ry, rz = 150.0, 179.0, 150.0

    # 찍는 z / 올라가는 z
    z_down = 68.0
    z_up = 200.0

    # ===== 경로 생성 =====
    dots = []
    for i in range(N_DOTS):
        t = i / (N_DOTS - 1)
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        dots.append(posx([x, y, z_down, rx, ry, rz]))

    path = []
    for i in range(N_DOTS - 1):
        p1 = dots[i]
        p2 = dots[i + 1]

        p1_up = posx([p1[0], p1[1], z_up,   rx, ry, rz])
        p2_up = posx([p2[0], p2[1], z_up,   rx, ry, rz])

        # 출발점 -> 출발점 z_up -> 도착점 z_up -> 도착점
        path += [p1, p1_up, p2_up, p2]

    # ===== movesx 실행 =====
    movesx(path, vel=VELOCITY, acc=ACC)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("movesx_dot_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
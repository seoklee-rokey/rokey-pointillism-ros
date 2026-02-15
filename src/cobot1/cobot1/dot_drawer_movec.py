# movec를 이용한 점찍기
# 속도 300, 높이 환경2에 맞춰 결정
# DR_MV_RA_OVERRIDE 넣음

import rclpy
import DR_init
import time

# ROS msg
from dot_msgs.msg import DotArray

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 300
ACC = 300

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    from DSR_ROBOT2 import (
        set_tool, set_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        set_robot_mode
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

 
def perform_task(node):
    """
    /draw_this (dot_msgs/DotArray)를 받아서 점묘 찍기 수행
    - 두산 방식: spin() 대신 while 루프 + spin_once()
    """
    from DSR_ROBOT2 import posx, movej, movel, movec, DR_MV_RA_OVERRIDE

    # ---- 동작 파라미터 ----
    JReady = [0, 0, 90, 0, 90, 0]   # start pose
    z_up = 74                     
    z_down = 68
    rx, ry, rz = 150, 179, 150      # fix

    latest_msg = None
    busy = False

    def cb(msg: DotArray):
        nonlocal latest_msg, busy
        if busy:
            return  # 그리는 중이면 무시(단순/안전)
        latest_msg = msg

    # 구독 (토픽명 고정)
    node.create_subscription(DotArray, "/draw_this", cb, 10)
    node.get_logger().info("Listening: /draw_this (dot_msgs/DotArray)")

    while rclpy.ok():
        # Doosan - 두산 스타일에서 콜백을 돌리기 위한 핵심
        rclpy.spin_once(node, timeout_sec=0.0)

        if latest_msg is None:
            time.sleep(0.01)
            continue

        busy = True
        msg = latest_msg
        latest_msg = None

        try:
            dots = msg.dots
            if len(dots) == 0:
                busy = False
                continue
            
            node.get_logger().info(f"Start drawing: {len(dots)} dots")
            
            # 시작 자세
            movej(JReady, vel=VELOCITY, acc=ACC) 
            # 첫 점 접근 (z_up)
            x0, y0 = float(dots[0].x), float(dots[0].y) 
            movel(posx([x0, y0, z_up, rx, ry, rz]), vel=VELOCITY, acc=ACC, )

            ############### logic ############

            # 점묘: (z_down 찍기 -> z_up 상승) 반복
            for i in range(len(dots) -1):
                x1,y1 = float(dots[i].x), float(dots[i].y)
                x2,y2 = float(dots[i+1].x), float(dots[i+1].y)

                xm = 0.5*(x1 + x2)
                ym = 0.5*(y1 + y2)

                via = posx([xm, ym, z_up, rx, ry, rz])
                tgt = posx([x2, y2, z_down, rx, ry, rz])
                
                movec(via, tgt, vel=VELOCITY, acc=ACC, ra=DR_MV_RA_OVERRIDE)
        
            ###################################

            movej(JReady, vel=VELOCITY, acc=ACC)
            node.get_logger().info("Drawing done")

        except Exception as e:
            node.get_logger().error(f"Drawing error: {e}")

        finally:
            busy = False


def main(args=None):
    rclpy.init(args=args)

    # Doosan - 두산 로직: create_node + DR_init에 node 지정
    node = rclpy.create_node("dot_drawer_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task(node)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

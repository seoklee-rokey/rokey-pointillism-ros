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

VELOCITY = 200
ACC = 200

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==============================
# 1. 로봇 초기화 
# ==============================
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

 
# ------------------------------
# 원본 movec 패턴을 함수로 분리
# ------------------------------
def movec_mid_to_next_down(x1, y1, x2, y2, z_up, z_down, rx, ry, rz, vel, acc, ra):
    from DSR_ROBOT2 import posx, movec

    xm = 0.5 * (x1 + x2)
    ym = 0.5 * (y1 + y2)

    via = posx([xm, ym, z_up,   rx, ry, rz])
    tgt = posx([x2, y2, z_down, rx, ry, rz])

    movec(via, tgt, vel=vel, acc=acc, ra=ra)

 
# ------------------------------
# v가 바뀔 때 : 들어올림 (나중에 펜교체로 발전)
# ------------------------------
def lift_high_at_xy(x, y, z_lift, rx, ry, rz, vel, acc):
    from DSR_ROBOT2 import posx, movel
    movel(posx([x, y, z_lift, rx, ry, rz]), vel=vel, acc=acc)

# ------------------------------
# 첫 점 접근 / 또는 v 변경 후 다음 색 시작점 접근
# ------------------------------
def go_xy_up(x, y, z_up, rx, ry, rz, vel, acc):
    from DSR_ROBOT2 import posx, movel
    movel(posx([x, y, z_up, rx, ry, rz]), vel=vel, acc=acc)



# ==============================
# 2. 점찍기 통합 로직 
# ==============================

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
    z_lift = z_up + 30   # 색 바뀔때 높이 -> 나중에 잡으러 가는 높이(위치)로 바뀔 예정

    rx, ry, rz = 150, 179, 150      # fix

    latest_msg = None
    busy = False

    # 이미 작업중일경우 받지 않기 위해.
    def cb(msg: DotArray):
        nonlocal latest_msg, busy
        if busy:
            return 
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
            
            node.get_logger().info(f"점찍기 시작 : 점 {len(dots)}개")
            node.get_logger().info(f"초기 위치로 이동")

            # 초기 자세
            movej(JReady, vel=VELOCITY, acc=ACC) 

            # 첫 점 접근 (z_up)
            x0, y0, v0 = float(dots[0].x), float(dots[0].y), int(dots[0].v)
            go_xy_up(x0, y0, z_up, rx, ry, rz, VELOCITY, ACC)

            prev_v = v0  # 색 변화 체크용

            # 첫 점 찍기 (movec 특성상 누락)
            movel(posx([x0, y0, z_down, rx, ry, rz]), vel=VELOCITY, acc=ACC)
            movel(posx([x0, y0, z_up, rx, ry, rz]), vel=VELOCITY, acc=ACC)

            

            ############### logic ############

            # 점묘: (z_down 찍기 -> z_up 상승) 반복
            for i in range(len(dots) -1):
                x1,y1,v1 = float(dots[i].x), float(dots[i].y), int(dots[i].v)
                x2,y2,v2 = float(dots[i+1].x), float(dots[i+1].y), int (dots[i+1].v)

                # 색이 바뀌었을 경우 들어올리기
                if i > 0 and v2 != prev_v:
                    node.get_logger().info(f"v change {prev_v} -> {v2}: lift")
                    lift_high_at_xy(x1, y1, z_lift, rx, ry, rz, VELOCITY, ACC)
                    go_xy_up(x2, y2, z_up, rx, ry, rz, VELOCITY, ACC)
                    prev_v = v2

                # 반복 찍기
                movec_mid_to_next_down(
                    x1, y1, x2, y2,
                    z_up=z_up, z_down=z_down,
                    rx=rx, ry=ry, rz=rz,
                    vel=VELOCITY, acc=ACC,
                    ra=DR_MV_RA_OVERRIDE
                )
        
            ###################################

            movej(JReady, vel=VELOCITY, acc=ACC)
            node.get_logger().info("Drawing done")

        except Exception as e:
            node.get_logger().error(f"Drawing error: {e}")

        finally:
            busy = False



def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dot_drawer_node", namespace=ROBOT_ID) # Doosan - 두산 로직: create_node + DR_init에 node 지정
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

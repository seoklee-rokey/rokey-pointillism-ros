#!/usr/bin/env python3

# 주제: 여러가지 색상으로 점묘화 만들기.

# 발전한 내용
# OFFSET 설정한 버전.
# 평탄화 완료

# 디버깅 중 : 색 바꾸고 강하게 찍는현상
# 디버깅 중 : Result 전송 못하고 터지는 현상


import rclpy
import DR_init
import time

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# ROS msg/action
from dot_msgs.action import DrawStipple
from dot_msgs.msg import DotArray

# ==============================
# 로봇 설정
# ==============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 이동 속도 및 가속도
VELOCITY = 300
ACC = 300

# ===============================
# 그리퍼용 설정값
# ===============================
PEN_Z_UP   = 120
PEN_TRAVEL_Z = PEN_Z_UP + 100 
PEN_Z_DOWN = 86
X_OFFSET = 1.3
Y_OFFSET = -0.4
PEN_PICK_TABLE = {
    # Right column (1~6)
    1:  (515.972+X_OFFSET, -302.237+Y_OFFSET, 82.827),
    2:  (516.289+X_OFFSET, -263.584+Y_OFFSET, 82.833),
    3:  (517.807+X_OFFSET, -224.931+Y_OFFSET, 82.839),
    4:  (516.924+X_OFFSET, -186.278+Y_OFFSET, 82.845),
    5:  (517.242+X_OFFSET, -147.625+Y_OFFSET, 82.851),
    6:  (517.559+X_OFFSET, -108.972+Y_OFFSET, 82.857),

    # Middle column (7~12)
    7:  (367.570+X_OFFSET, -300.730+Y_OFFSET, 83.130),
    8:  (367.863+X_OFFSET, -262.382+Y_OFFSET, 82.899),
    9:  (367.156+X_OFFSET, -224.035+Y_OFFSET, 82.669),
    10: (367.449+X_OFFSET, -185.688+Y_OFFSET, 82.438),
    11: (367.742+X_OFFSET, -147.341+Y_OFFSET, 82.208),
    12: (367.035+X_OFFSET, -108.993+Y_OFFSET, 81.977),

    # Left column (13~18)
    13: (228.367+X_OFFSET, -299.322+Y_OFFSET, 82.133),
    14: (228.136+X_OFFSET, -261.581+Y_OFFSET, 82.144),
    15: (228.005+X_OFFSET, -223.339+Y_OFFSET, 82.855),
    16: (228.373+X_OFFSET, -185.098+Y_OFFSET, 82.566),
    17: (228.442+X_OFFSET, -147.056+Y_OFFSET, 82.277),
    18: (228.511+X_OFFSET, -109.015+Y_OFFSET, 81.989),
}

# 디지털 출력상태
ON,OFF = 1,0
# set
DO_GRIP = 1       # set_digital_output(1, ON)  -> Grip
DO_RELEASE = 2    # set_digital_output(2, ON)  -> Release
# get
DI_GRIP = 1       # get_digital_input(1)       -> Grip 완료
DI_RELEASE = 2    # get_digital_input(2)       -> Release 완료

def wait_digital_input(sig_num, wait_fn, get_di_fn):
    while not get_di_fn(sig_num):
        wait_fn(0.1)

def gripper_release(set_do_fn, wait_fn, get_di_fn):
    # Release 동작
    set_do_fn(DO_RELEASE, ON)
    set_do_fn(DO_GRIP, OFF)
    wait_digital_input(DI_RELEASE, wait_fn, get_di_fn)

def gripper_grip(set_do_fn, wait_fn, get_di_fn):
    # Grip 동작
    set_do_fn(DO_GRIP, ON)
    set_do_fn(DO_RELEASE, OFF)
    wait_digital_input(DI_GRIP, wait_fn, get_di_fn)



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
# 원본 movec 패턴을 함수로 분리 # 변경 로직 적용
# ------------------------------
def movec_mid_to_next_down(x2, y2,
                          z_up, z_down, rx, ry, rz, vel, acc, ra):
    from DSR_ROBOT2 import posx, movec

    via = posx([x2, y2, z_up,   rx, ry, rz]) # 일단 72로 정해둠.
    tgt = posx([x2, y2, z_down, rx, ry, rz])

    movec(via, tgt, vel=vel, acc=acc, ra=0) # 블렌딩 없애보기 # 원래 : ra


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


# ------------------------------
# 펜 변경 로직
# ------------------------------
def change_pen_by_v(prev_v: int, target_v: int, rx, ry, rz, vel, acc):
    """
    펜교체 & 최소 펜 pick
    - prev_v가 None이면: target_v 펜만 pick
    - prev_v != target_v이면: prev_v 내려놓고 target_v pick
    - 스테이션 간 이동은 PEN_TRAVEL_Z로 안전하게 이동
    """
    from DSR_ROBOT2 import posx, movel, set_digital_output, get_digital_input, wait

    # 예외처리) Pick table에 정의 안 된 색상이면 펜교체 스킾하고 그냥 계속 찍음
    if target_v not in PEN_PICK_TABLE:
        return
    
    # 예외처리) prev == target면 아무것도 안 함
    if prev_v == target_v:
        return

    # 최초엔 gripper open 보장
    if prev_v is None:
        gripper_release(set_digital_output, wait, get_digital_input)

        pick_x, pick_y, pick_z = PEN_PICK_TABLE[target_v]
        # 스테이션 접근(충돌 방지 높이)
        movel(posx([pick_x, pick_y, PEN_TRAVEL_Z, rx, ry, rz]), vel=vel, acc=acc)
        # movel(posx([pick_x, pick_y, PEN_Z_UP,      rx, ry, rz]), vel=vel, acc=acc)
        movel(posx([pick_x, pick_y, pick_z,    rx, ry, rz]), vel=vel, acc=acc)
        gripper_grip(set_digital_output, wait, get_digital_input)
        # movel(posx([pick_x, pick_y, PEN_Z_UP,      rx, ry, rz]), vel=vel, acc=acc)
        movel(posx([pick_x, pick_y, PEN_TRAVEL_Z,  rx, ry, rz]), vel=vel, acc=acc)
        return
    
    # 예외처리) prev 정의 안 된 경우(테이블에 없으면) -> 그냥 pick만
    if prev_v not in PEN_PICK_TABLE:
        change_pen_by_v(None, target_v, rx, ry, rz, vel, acc)
        return
    
    put_x, put_y, put_z   = PEN_PICK_TABLE[prev_v]
    pick_x, pick_y, pick_z = PEN_PICK_TABLE[target_v]

    # prev_v 펜 내려놓기
    movel(posx([put_x, put_y, PEN_TRAVEL_Z, rx, ry, rz]), vel=vel, acc=acc)
    # movel(posx([put_x, put_y, PEN_Z_UP,     rx, ry, rz]), vel=vel, acc=acc)
    movel(posx([put_x, put_y, put_z,   rx, ry, rz]), vel=vel, acc=acc)
    gripper_release(set_digital_output, wait, get_digital_input)
    # movel(posx([put_x, put_y, PEN_Z_UP,     rx, ry, rz]), vel=vel, acc=acc)
    movel(posx([put_x, put_y, PEN_TRAVEL_Z, rx, ry, rz]), vel=vel, acc=acc)

    # target_v 펜 집기+ 들어올리기 
    movel(posx([pick_x, pick_y, PEN_TRAVEL_Z, rx, ry, rz]), vel=vel, acc=acc)
    # movel(posx([pick_x, pick_y, PEN_Z_UP,     rx, ry, rz]), vel=vel, acc=acc)
    movel(posx([pick_x, pick_y, pick_z ,   rx, ry, rz]), vel=vel, acc=acc)
    gripper_grip(set_digital_output, wait, get_digital_input)
    # movel(posx([pick_x, pick_y, PEN_Z_UP,     rx, ry, rz]), vel=vel, acc=acc)
    movel(posx([pick_x, pick_y, PEN_TRAVEL_Z, rx, ry, rz]), vel=vel, acc=acc)


# ==============================
# Action Server Node
# ==============================
class DotDrawerAction(Node):
    def __init__(self):
        super().__init__("dot_drawer_action", namespace=ROBOT_ID)
        # planner config (kept minimal for new planner module)

        self._busy = False

        self._action_server = ActionServer(
            self,
            DrawStipple,
            "/draw_stipple",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("ActionServer ready: /draw_stipple (dot_msgs/DrawStipple)")


    def goal_callback(self, goal_request: DrawStipple.Goal):
        # 동시에 여러 개 goal 받지 않기
        if self._busy:
            self.get_logger().warn("Reject goal: busy")
            return GoalResponse.REJECT

        # 예외처리) 빈 데이터 거절
        if goal_request.data is None or len(goal_request.data.dots) == 0:
            self.get_logger().warn("Reject goal: empty dots")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle):
        # 움직이는 도중 cancel을 받으면, 루프에서 확인해서 안전 종료하도록 구성
        self.get_logger().warn("Cancel requested")
        return CancelResponse.ACCEPT


    def _publish_feedback(self, goal_handle, done: int, total: int, current_v: int):
        fb = DrawStipple.Feedback()
        if total <= 0:
            fb.percent = 0.0
        else:
            fb.percent = float(done) / float(total) * 100.0
        fb.current_v = int(current_v)
        fb.current_index = int(done)
        goal_handle.publish_feedback(fb) # 

   

    def execute_callback(self, goal_handle):
        from DSR_ROBOT2 import posx, movej, movel, DR_MV_RA_OVERRIDE

        # ---------------------------
        # 파라미터 설정 및 디버깅
        # ---------------------------
        self._busy = True
        t0 = time.time()

        req = goal_handle.request
        incoming: DotArray = req.data  # incoming이 좌표데이터 리스트임
        
        # 디버깅용
        self.get_logger().info(
            f"[Goal Received] dot_count={len(incoming.dots)}"
        )
        for i, d in enumerate(incoming.dots[:3]):
            self.get_logger().info(
                f"[Dot {i}] x={d.x}, y={d.y}, v={d.v}"
            )

        # 동작 파라미터 
        JReady = [0, 0, 90, 0, 90, 0]
        z_up = 73
        z_down = 67
        z_lift = z_up + 30
        rx, ry, rz = 150, 179, 150

        # 순서 받아오기
        try:
            point_list = [(d.x, d.y, d.v) for d in incoming.dots]
            plan = point_list
        except Exception as e:
            self.get_logger().error(f"Planner error: {e}")
            goal_handle.abort()
            self._busy = False
            res = DrawStipple.Result()
            res.success = False
            return res
        
        total = len(plan)
        
        # 예외처리) total 0일경우 종료
        if total == 0:
            self.get_logger().warn("No dots after planning.")
            goal_handle.succeed()
            self._busy = False
            res = DrawStipple.Result()
            res.success = True
            return res

        current_pen = None  # 지금 잡고 있는 펜(v) # 안잡고 있다고 이니시에이트.
        
        
        # ==================================
        # 실제 동작 실행 코드
        # ==================================
        try:
            movej(JReady, vel=100, acc=100)

            x0, y0, v0 = plan[0]
            v0 = int(v0)
            
            # 첫 펜 잡기: v0에 해당하는 펜을 스테이션에서 집음
            change_pen_by_v(current_pen, v0, rx, ry, rz, VELOCITY, ACC)
            current_pen = v0

            go_xy_up(x0, y0, PEN_TRAVEL_Z, rx, ry, rz, VELOCITY, ACC)

            # 첫 점 찍기
            movel(posx([x0, y0, z_up, rx, ry, rz]), vel=VELOCITY, acc=ACC)
            movel(posx([x0, y0, z_down, rx, ry, rz]), vel=VELOCITY, acc=ACC)
            movel(posx([x0, y0, z_up, rx, ry, rz]), vel=VELOCITY, acc=ACC)
            
            # 디버깅
            self.get_logger().info(f"[DRAW] 1/{total} 번째 점 완료 (v={current_pen})")

            # feedback (첫 점 완료)
            self._publish_feedback(goal_handle, 1, total, current_pen)

            for i in range(total - 1):
                # cancle request가 들어왔을 경우.
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn("Canceled by client")
                    # 안전하게 들어올리고 원위치.
                    try:
                        x_cur, y_cur, _ = plan[i]
                        lift_high_at_xy(x_cur, y_cur, z_lift, rx, ry, rz, VELOCITY, ACC)
                        movej(JReady, vel=VELOCITY, acc=ACC)
                    except Exception:
                        pass
                    goal_handle.canceled()
                    self._busy = False
                    res = DrawStipple.Result()
                    res.success = False
                    return res

                # 여기서부터 찍기코드
                x1, y1, v1 = plan[i]
                x2, y2, v2 = plan[i + 1]
                v2 = int(v2)

                # 색이 바뀔 경우 펜 변경
                if i > 0 and v2 != current_pen:
                    self.get_logger().info(f"v change {current_pen} -> {v2}: lift + pen swap")
                    
                    lift_high_at_xy(x1, y1, PEN_TRAVEL_Z, rx, ry, rz, VELOCITY, ACC)  # 그림 위치에서 먼저 충분히 상승
                    change_pen_by_v(current_pen, v2, rx, ry, rz, VELOCITY, ACC) # 스테이션에서 내려놓고 새로 집기(안전높이 포함)
                    current_pen = v2

                    go_xy_up(x2, y2, PEN_TRAVEL_Z, rx, ry, rz, VELOCITY, ACC) 
                    go_xy_up(x2, y2, z_up, rx, ry, rz, VELOCITY, ACC)                 # 다음 점 시작점 접근
                    movel(posx([x2, y2, z_down+0.4, rx, ry, rz]), vel=VELOCITY, acc=ACC)  # 직접 찍기
                    
                    done = i + 2
                    self._publish_feedback(goal_handle, done, total, current_pen)

                    continue # 아래의 movec를 이번 턴에 안타게 함

                movec_mid_to_next_down(
                    x2, y2,
                    z_up=z_up, z_down=z_down,
                    rx=rx, ry=ry, rz=rz,
                    vel=VELOCITY, acc=ACC,
                    ra=DR_MV_RA_OVERRIDE
                )

                # feedback throttle: 매 점마다.
                done = i + 2

                ### 디버깅용 ###
                self.get_logger().info(f"[DRAW] {done}/{total} 번째 점 완료 (v={current_pen})")
                ##############

                self._publish_feedback(goal_handle, done, total, current_pen)

            # movej(JReady, vel=VELOCITY, acc=ACC)
            self.get_logger().info(f"Drawing done ({total} dots). elapsed={time.time()-t0:.2f}s")

            goal_handle.succeed()
            res = DrawStipple.Result()
            res.success = True
            return res

        except Exception as e:
            self.get_logger().error(f"Drawing error: {e}")
            goal_handle.abort()
            res = DrawStipple.Result()
            res.success = False
            return res

        # =========================================
        # 작업 종료 시: 현재 펜을 스테이션에 내려놓고 마무리 
        # =========================================

        finally: 
            try:
                if current_pen is not None and current_pen in PEN_PICK_TABLE:
                    from DSR_ROBOT2 import posx, movel, set_digital_output, get_digital_input, wait

                    # (안전) 현재 위치에서 충분히 들어올리기 (펜 들고 이동 시 충돌 방지)
                    try:
                        # 마지막 점 좌표를 알고 있으면 거기서 lift
                        x_end, y_end, _ = plan[-1]
                        lift_high_at_xy(x_end, y_end, PEN_TRAVEL_Z, rx, ry, rz, VELOCITY, ACC)
                    except Exception:
                        # plan이 없거나 좌표 못 얻으면 생략
                        pass

                    put_x, put_y, put_z = PEN_PICK_TABLE[current_pen]

                    # 스테이션 접근(충돌 방지 높이)
                    movel(posx([put_x, put_y, PEN_TRAVEL_Z, rx, ry, rz]), vel=VELOCITY, acc=ACC)
                    movel(posx([put_x, put_y, PEN_Z_UP,     rx, ry, rz]), vel=VELOCITY, acc=ACC)
                    movel(posx([put_x, put_y, put_z,   rx, ry, rz]), vel=VELOCITY, acc=ACC)

                    # 펜 내려놓기
                    gripper_release(set_digital_output, wait, get_digital_input)

                    # 다시 안전 높이로
                    movel(posx([put_x, put_y, PEN_Z_UP,     rx, ry, rz]), vel=VELOCITY, acc=ACC)
                    movel(posx([put_x, put_y, PEN_TRAVEL_Z, rx, ry, rz]), vel=VELOCITY, acc=ACC)
                    movej(JReady, vel=100, acc=100)

                    self.get_logger().info(f"[END] Pen #{current_pen} placed back to station.")
                    current_pen = None

            except Exception as e:
                self.get_logger().warn(f"[END] Failed to place pen back: {e}")

            self._busy = False


def main(args=None):
    rclpy.init(args=args)

    node = DotDrawerAction()
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

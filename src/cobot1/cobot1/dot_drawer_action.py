#!/usr/bin/env python3

# 1차 개발완료 정상작동 코드

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

VELOCITY = 200
ACC = 200

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# pub_sm과 동일 작업영역 기본값
X_LEFT = 320
Y_TOP = 0
X_RIGHT = 500
Y_BOTTOM = 120


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
def movec_mid_to_next_down(x1, y1, x2, y2,
                          z_up, z_down, rx, ry, rz, vel, acc, ra):
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

        # 빈 데이터 거절
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
        goal_handle.publish_feedback(fb)


    def execute_callback(self, goal_handle):
        from DSR_ROBOT2 import posx, movej, movel, DR_MV_RA_OVERRIDE

        self._busy = True
        t0 = time.time()

        req = goal_handle.request
        incoming: DotArray = req.data
        
        ### 디버깅용 ###
        self.get_logger().info(
            f"[Goal Received] dot_count={len(incoming.dots)}"
        )
        for i, d in enumerate(incoming.dots[:3]):
            self.get_logger().info(
                f"[Dot {i}] x={d.x}, y={d.y}, v={d.v}"
            )
        ###############

        # ---- 동작 파라미터(기존 유지) ----
        JReady = [0, 0, 90, 0, 90, 0]
        
        z_up = 74
        z_down = 68
        z_lift = z_up + 30

        rx, ry, rz = 150, 179, 150

        # 1) (v별) 순서 최적화 + 작업영역 좌표 변환
        try:
            point_list = [(d.x, d.y, d.v) for d in incoming.dots]
            ########### 어떻게 입력할지 사용하는 지점 ###################################### 중요! ###############
            plan = point_list
        except Exception as e:
            self.get_logger().error(f"Planner error: {e}")
            goal_handle.abort()
            self._busy = False
            res = DrawStipple.Result()
            res.success = False
            return res

        total = len(plan)
        if total == 0:
            self.get_logger().warn("No dots after planning.")
            goal_handle.succeed()
            self._busy = False
            res = DrawStipple.Result()
            res.success = True
            return res


        # 2) 실제 점찍기 실행 (기존 코드 최대 유지)
        try:
            movej(JReady, vel=VELOCITY, acc=ACC)

            x0, y0, v0 = plan[0]
            go_xy_up(x0, y0, z_up, rx, ry, rz, VELOCITY, ACC)

            prev_v = int(v0)

            # 첫 점 찍기
            movel(posx([x0, y0, z_down, rx, ry, rz]), vel=VELOCITY, acc=ACC)
            movel(posx([x0, y0, z_up, rx, ry, rz]), vel=VELOCITY, acc=ACC)
            
            ### 디버깅용 ###
            self.get_logger().info(f"[DRAW] 1/{total} 번째 점 완료 (v={prev_v})")
            ###############

            # feedback (첫 점 완료)
            self._publish_feedback(goal_handle, 1, total, prev_v)

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

                # 색이 바뀔 경우 들어올리기
                if i > 0 and v2 != prev_v:
                    self.get_logger().info(f"v change {prev_v} -> {v2}: lift")
                    lift_high_at_xy(x1, y1, z_lift, rx, ry, rz, VELOCITY, ACC)
                    go_xy_up(x2, y2, z_up, rx, ry, rz, VELOCITY, ACC)
                    prev_v = v2

                movec_mid_to_next_down(
                    x1, y1, x2, y2,
                    z_up=z_up, z_down=z_down,
                    rx=rx, ry=ry, rz=rz,
                    vel=VELOCITY, acc=ACC,
                    ra=DR_MV_RA_OVERRIDE
                )

                # feedback throttle: 매 점마다.
                done = i + 2

                ### 디버깅용 ###
                self.get_logger().info(f"[DRAW] {done}/{total} 번째 점 완료 (v={prev_v})")
                ##############

                self._publish_feedback(goal_handle, done, total, prev_v)

            movej(JReady, vel=VELOCITY, acc=ACC)
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

        finally:
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

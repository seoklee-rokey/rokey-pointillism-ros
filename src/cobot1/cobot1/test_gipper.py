#!/usr/bin/env python3
import time
import rclpy
import DR_init
from rclpy.node import Node

# ===== Robot basic config =====
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP  = "GripperDA_v1"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ===== Speed =====
VELOCITY = 60
ACC = 60

# ===== Gripper I/O =====
ON, OFF = 1, 0
DO_GRIP = 1
DO_RELEASE = 2
DI_GRIP = 1
DI_RELEASE = 2

# ===== Pen positions =====
PEN_Z_UP   = 120
PEN_Z_DOWN = 86
PEN_PICK_TABLE = {
    1: (392, 215),
    2: (392, 242),
    3: (392, 269),
}

# ===== Tool pose (IMPORTANT: set to your real orientation) =====
RX, RY, RZ = 12.4, -178.0, 11.28   # <-- 너 환경에 맞게 수정

def wait_di(sig_num, wait_fn, get_di_fn, timeout_s=3.0):
    """DI가 1 될 때까지 대기. timeout 걸면 False."""
    t0 = time.time()
    while not get_di_fn(sig_num):
        if time.time() - t0 > timeout_s:
            return False
        wait_fn(0.05)
    return True

def gripper_release(set_do_fn, wait_fn, get_di_fn):
    set_do_fn(DO_RELEASE, ON)
    set_do_fn(DO_GRIP, OFF)
    ok = wait_di(DI_RELEASE, wait_fn, get_di_fn)
    if not ok:
        raise RuntimeError("Release timeout (DI_RELEASE not set)")

def gripper_grip(set_do_fn, wait_fn, get_di_fn):
    set_do_fn(DO_GRIP, ON)
    set_do_fn(DO_RELEASE, OFF)
    ok = wait_di(DI_GRIP, wait_fn, get_di_fn)
    if not ok:
        raise RuntimeError("Grip timeout (DI_GRIP not set)")

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

def pos_at(x, y, z):
    """(x,y,z,RX,RY,RZ) 포즈 생성"""
    from DSR_ROBOT2 import posx
    return posx(x, y, z, RX, RY, RZ)

def move_xy_z(movel_fn, x, y, z, vel=VELOCITY, acc=ACC):
    """직선이동 movel"""
    p = pos_at(x, y, z)
    movel_fn(p, vel=vel, acc=acc)

def pick_pen(movel_fn, set_do_fn, wait_fn, get_di_fn, pen_no):
    """펜 집기: UP -> DOWN -> GRIP -> UP"""
    x, y = PEN_PICK_TABLE[pen_no]
    move_xy_z(movel_fn, x, y, PEN_Z_UP)
    move_xy_z(movel_fn, x, y, PEN_Z_DOWN)
    gripper_grip(set_do_fn, wait_fn, get_di_fn)
    move_xy_z(movel_fn, x, y, PEN_Z_UP)

def place_pen_back(movel_fn, set_do_fn, wait_fn, get_di_fn, pen_no):
    """펜 내려놓기: UP -> DOWN -> RELEASE -> UP"""
    x, y = PEN_PICK_TABLE[pen_no]
    move_xy_z(movel_fn, x, y, PEN_Z_UP)
    move_xy_z(movel_fn, x, y, PEN_Z_DOWN)
    gripper_release(set_do_fn, wait_fn, get_di_fn)
    move_xy_z(movel_fn, x, y, PEN_Z_UP)

def main(args=None):
    rclpy.init(args=args)
    node = Node("pen_pick_demo", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import set_digital_output, get_digital_input, wait, movel

    try:
        initialize_robot()

        # 1) 그리퍼 열기
        node.get_logger().info("1) Gripper release (open)")
        gripper_release(set_digital_output, wait, get_digital_input)

        # 2~3) 1번 펜 집기 (DOWN에서 잡고 UP으로 들어올림)
        node.get_logger().info("2~3) Pick pen #1")
        pick_pen(movel, set_digital_output, wait, get_digital_input, pen_no=1)

        # 4) 1번 펜 다시 내려놓기
        node.get_logger().info("4) Place pen #1 back")
        place_pen_back(movel, set_digital_output, wait, get_digital_input, pen_no=1)

        # 5~6) 2번 펜 집고 들어올리기
        node.get_logger().info("5~6) Pick pen #2")
        pick_pen(movel, set_digital_output, wait, get_digital_input, pen_no=2)

        node.get_logger().info("Done.")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"ERROR: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
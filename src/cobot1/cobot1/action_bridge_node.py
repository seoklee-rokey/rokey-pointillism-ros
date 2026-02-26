# action_bridge_node.py  (FULL REWRITE: resume + watchdog + current_index done_count)
import uuid
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from dot_msgs.action import DrawStipple
from dot_msgs.msg import Dot, DotArray

from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn


# ===============================
# HTTP 데이터 모델
# ===============================
class RunRequest(BaseModel):
    token: str
    dots: list  # [[x,y,v], ...]


class ResumeRequest(BaseModel):
    job_id: str


class CancelRequest(BaseModel):
    job_id: str


# ===============================
# ETA 계산 유틸
# ===============================
def count_swaps_from_dots(dots: list) -> int:
    """
    정의 A: '교체'만 센다.
    즉, v가 바뀌는 횟수만 카운트 (첫 펜 집기는 교체로 안침)
    dots: [[x,y,v], ...]
    """
    if not dots:
        return 0

    swaps = 0
    prev_v = int(dots[0][2])
    for d in dots[1:]:
        v = int(d[2])
        if v != prev_v:
            swaps += 1
            prev_v = v
    return swaps


# ===============================
# ROS Bridge Node
# ===============================
class ActionBridge(Node):
    def __init__(self):
        super().__init__("action_bridge")
        self._action_client = ActionClient(self, DrawStipple, "/draw_stipple")
        self.jobs = {}  # job_id -> 상태 저장

    def _safe_get_job(self, job_id: str):
        return self.jobs.get(job_id)

    def send_goal(self, job_id: str, dots_norm: list):
        """
        dots_norm: [[x,y,v], ...]  (이미 로봇이 쓰는 좌표 체계로 들어온다고 가정)
        """
        job = self._safe_get_job(job_id)
        if not job:
            return

        # 액션 서버 대기 (resume 시에는 timeout 짧게 확인하도록 /resume에서 먼저 체크함)
        self._action_client.wait_for_server()

        goal_msg = DrawStipple.Goal()
        goal_msg.data = DotArray()

        for x, y, v in dots_norm:
            dot = Dot()
            dot.x = float(x)
            dot.y = float(y)
            dot.v = int(v)
            goal_msg.data.dots.append(dot)

        # 상태 업데이트
        with job["lock"]:
            job["state"] = "PENDING"
            job["done"] = False
            job["error"] = None
            job["message"] = "goal 전송 중..."
            job["last_feedback_ts"] = time.time()

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self.feedback_callback(job_id, fb),
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(job_id, future)
        )

    def goal_response_callback(self, job_id, future):
        job = self._safe_get_job(job_id)
        if not job:
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            with job["lock"]:
                job["state"] = "FAILED"
                job["done"] = True
                job["error"] = "GOAL_SEND_ERROR"
                job["message"] = f"goal 전송 실패: {e}"
            return

        if not goal_handle.accepted:
            with job["lock"]:
                job["state"] = "REJECTED"
                job["done"] = True
                job["error"] = "GOAL_REJECTED"
                job["message"] = "goal이 거부되었습니다."
            return

        with job["lock"]:
            job["state"] = "RUNNING"
            job["_goal_handle"] = goal_handle
            job["message"] = "작업 진행 중..."

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.result_callback(job_id, f)
        )

    def feedback_callback(self, job_id, feedback_msg):
        """
        ✅ 핵심:
        - 액션 서버 feedback에 current_index가 들어오면 그걸 done_dots로 사용
        - percent 기반 추정은 제거
        """
        feedback = feedback_msg.feedback
        job = self._safe_get_job(job_id)
        if not job:
            return

        # 액션 서버에서 온 값 (필드 없을 수도 있으니 getattr)
        percent = float(getattr(feedback, "percent", 0.0))
        current_v = int(getattr(feedback, "current_v", 0))
        current_index = int(getattr(feedback, "current_index", 0))  # ✅ done_count 역할

        with job["lock"]:
            job["last_feedback_ts"] = time.time()
            job["percent"] = percent
            job["current_v"] = current_v

            total_dots = int(job.get("total_dots", 0))

            # ✅ done_dots는 current_index 그대로 (단조 증가 보장)
            done_dots = max(0, min(current_index, total_dots))
            prev_done = int(job.get("done_dots", 0))
            if done_dots < prev_done:
                done_dots = prev_done
            job["done_dots"] = done_dots

            # done_swaps 추정 (기존 로직 유지)
            if job["last_v"] is None:
                job["last_v"] = current_v
            else:
                if current_v != job["last_v"]:
                    job["done_swaps"] = int(job.get("done_swaps", 0)) + 1
                    job["last_v"] = current_v

    def result_callback(self, job_id, future):
        job = self._safe_get_job(job_id)
        if not job:
            return

        try:
            wrapped = future.result()
            result = wrapped.result
            status = getattr(wrapped, "status", None)
        except Exception as e:
            with job["lock"]:
                job["done"] = True
                job["state"] = "FAILED"
                job["error"] = "RESULT_ERROR"
                job["message"] = f"결과 수신 실패: {e}"
            return

        with job["lock"]:
            job["done"] = True

            # ROS2 GoalStatus: 2=CANCELED
            if status == 2:
                job["state"] = "CANCELED"
                job["message"] = "작업이 취소되었습니다."
            else:
                if getattr(result, "success", False):
                    job["state"] = "SUCCEEDED"
                    job["message"] = "작업 완료"
                    # 완료면 done_dots를 total로 맞춰주는 게 UI 안정적
                    job["done_dots"] = int(job.get("total_dots", 0))
                    job["percent"] = 100.0
                else:
                    job["state"] = "FAILED"
                    job["error"] = "ACTION_FAILED"
                    job["message"] = "작업 실패"


# ===============================
# FastAPI 서버
# ===============================
app = FastAPI()
bridge_node: ActionBridge | None = None


# ETA 파라미터(대충값)
DOT_SEC = 0.4     # 점 1개 찍는 시간(초)
SWAP_SEC = 20.0   # 펜 교체 1회 시간(초)

# Watchdog
WATCHDOG_SEC = 15.0        # 5초 동안 feedback 없으면 다운으로 판단
WATCHDOG_TICK_SEC = 0.5
WD_CONFIRM_MISSES = 3   # 연속 3번 서버 안 보이면 확정

def watchdog_loop():
    global bridge_node
    while True:
        time.sleep(WATCHDOG_TICK_SEC)
        if bridge_node is None:
            continue

        now = time.time()

        for job_id, job in list(bridge_node.jobs.items()):
            lock = job.get("lock")
            if lock is None:
                continue

            with lock:
                if job.get("state") != "RUNNING":
                    job["wd_miss"] = 0
                    continue

                last = float(job.get("last_feedback_ts", 0.0))
                if last <= 0:
                    continue

                # 1️⃣ 피드백 끊겼는지 (의심 단계)
                if (now - last) <= WATCHDOG_SEC:
                    job["wd_miss"] = 0
                    continue

            # 🔵 lock 밖에서 서버 alive 체크 (중요)
            #server_alive = bridge_node._action_client.wait_for_server(timeout_sec=0.0)
            server_alive = bridge_node._action_client.server_is_ready()

            with lock:
                if job.get("state") != "RUNNING":
                    job["wd_miss"] = 0
                    continue

                if server_alive:
                    # 서버는 살아있음 → 느린 동작
                    job["wd_miss"] = 0
                    continue

                # 서버 안 보임
                job["wd_miss"] = int(job.get("wd_miss", 0)) + 1

                if job["wd_miss"] >= WD_CONFIRM_MISSES:
                    job["state"] = "FAILED"
                    job["done"] = True
                    job["error"] = "ACTION_SERVER_DOWN"
                    job["message"] = "액션 서버가 중단되었습니다. 서버를 다시 켠 뒤 재개를 눌러주세요."


@app.post("/run")
def run_robot(req: RunRequest):
    """
    job 생성 + 즉시 goal 전송
    """
    if bridge_node is None:
        return {"ok": False, "error": "bridge not ready"}

    job_id = str(uuid.uuid4())

    total_dots = len(req.dots)
    total_swaps = count_swaps_from_dots(req.dots)

    bridge_node.jobs[job_id] = {
        "token": req.token,

        "state": "PENDING",
        "percent": 0.0,
        "current_v": 0,
        "done": False,
        "message": "대기 중...",

        # ✅ resume 필수
        "dots": req.dots,
        "total_dots": total_dots,
        "done_dots": 0,

        # ETA
        "total_swaps": total_swaps,
        "done_swaps": 0,
        "last_v": None,

        # watchdog
        "last_feedback_ts": time.time(),
        "wd_miss": 0,   # ✅ 추가 (연속 실패 카운트)

        # runtime
        "_goal_handle": None,
        "error": None,

        # 동시 호출 보호
        "lock": threading.Lock(),
    }

    bridge_node.send_goal(job_id, req.dots)
    return {"ok": True, "job_id": job_id}


@app.get("/status/{job_id}")
def get_status(job_id: str):
    if bridge_node is None:
        return {"ok": False, "error": "bridge not ready"}

    if job_id not in bridge_node.jobs:
        return {"ok": False, "error": "invalid job_id"}

    data = bridge_node.jobs[job_id]

    with data["lock"]:
        total_dots = int(data.get("total_dots", 0))
        done_dots = int(data.get("done_dots", 0))
        total_swaps = int(data.get("total_swaps", 0))
        done_swaps = int(data.get("done_swaps", 0))

        remain_dots = max(0, total_dots - done_dots)
        remain_swaps = max(0, total_swaps - done_swaps)

        remain_sec = remain_dots * DOT_SEC + remain_swaps * SWAP_SEC
        eta_min = int(math.ceil(remain_sec / 60.0)) if remain_sec > 0 else 0

        return {
            "ok": True,
            "state": data.get("state"),
            "percent": float(data.get("percent", 0.0)),
            "current_v": int(data.get("current_v", 0)),
            "done": bool(data.get("done", False)),
            "message": data.get("message", ""),
            "error": data.get("error"),

            # 진행/ETA
            "total_dots": total_dots,
            "done_dots": done_dots,
            "total_swaps": total_swaps,
            "done_swaps": done_swaps,
            "eta_min": eta_min,
        }


@app.post("/resume")
def resume_robot(req: ResumeRequest):
    """
    사람이 액션 서버를 다시 켠 뒤 호출하면,
    done_dots 이후부터 remaining만 goal로 다시 전송
    """
    if bridge_node is None:
        return {"ok": False, "error": "bridge not ready"}

    job_id = req.job_id
    if job_id not in bridge_node.jobs:
        return {"ok": False, "error": "invalid job_id"}

    job = bridge_node.jobs[job_id]

    # 액션 서버가 아직 안 살아있으면 바로 실패 반환 (팝업 유지)
    #if not bridge_node._action_client.wait_for_server(timeout_sec=0.2):
    if not bridge_node._action_client.server_is_ready():
        with job["lock"]:
            job["state"] = "FAILED"
            job["done"] = True
            job["error"] = "ACTION_SERVER_DOWN"
            job["message"] = "아직 액션 서버가 실행 중이 아닙니다."
        return {"ok": False, "error": "ACTION_SERVER_DOWN"}

    with job["lock"]:
        total = int(job.get("total_dots", 0))
        done = int(job.get("done_dots", 0))
        done = max(0, min(done, total))

        if done >= total:
            job["state"] = "SUCCEEDED"
            job["done"] = True
            job["error"] = None
            job["message"] = "이미 완료된 작업입니다."
            return {"ok": True, "state": job["state"], "done_dots": done, "total_dots": total}

        # running 중 resume 방지(중복 전송 방지)
        if job.get("state") == "RUNNING":
            return {"ok": False, "error": "ALREADY_RUNNING"}

        remaining = job["dots"][done:]

        # 상태 리셋 (done_dots는 유지)
        job["state"] = "PENDING"
        job["done"] = False
        job["error"] = None
        job["message"] = f"재개 중... ({done}/{total})"
        job["last_feedback_ts"] = time.time()
        job["_goal_handle"] = None

    bridge_node.send_goal(job_id, remaining)
    return {"ok": True, "state": "RESUMED", "from": done, "total": total}


@app.post("/cancel")
def cancel_robot(req: CancelRequest):
    if bridge_node is None:
        return {"ok": False, "error": "bridge not ready"}

    job_id = req.job_id
    if job_id not in bridge_node.jobs:
        return {"ok": False, "error": "invalid job_id"}

    data = bridge_node.jobs[job_id]
    with data["lock"]:
        gh = data.get("_goal_handle")
        if gh is None:
            return {"ok": False, "error": "goal_handle not ready yet"}

        try:
            gh.cancel_goal_async()
            data["state"] = "CANCEL_REQUESTED"
            data["message"] = "취소 요청 중..."
            return {"ok": True, "state": data["state"]}
        except Exception as e:
            return {"ok": False, "error": f"cancel failed: {e}"}


# ===============================
# 실행
# ===============================
def main():
    global bridge_node

    rclpy.init()
    bridge_node = ActionBridge()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(bridge_node,),
        daemon=True
    )
    ros_thread.start()

    wd_thread = threading.Thread(target=watchdog_loop, daemon=True)
    wd_thread.start()

    uvicorn.run(app, host="0.0.0.0", port=8089)


if __name__ == "__main__":
    main()
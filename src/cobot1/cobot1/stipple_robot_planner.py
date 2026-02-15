# stipple_robot_planner.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Iterable
import math

# dot_msgs imports (로봇 패키지에서 사용)
# from dot_msgs.msg import Dot, DotArray

@dataclass
class Workspace:
    x_left: float = 320.0
    x_right: float = 500.0
    y_top: float = 0.0
    y_bottom: float = 120.0
    invert_y: bool = False        # 필요하면 True
    clamp: bool = True            # 작업영역 밖 값이 오면 클램프


@dataclass
class InputSpec:
    normalized_xy: bool = True    # True면 x,y가 0~1
    img_w: float = 1.0            # normalized면 의미 없음
    img_h: float = 1.0            # normalized면 의미 없음


@dataclass
class PlannerConfig:
    # NN grid 크기 (로봇 좌표 단위)
    cell_size: float = 5.0
    # 그룹 간 선택 기준 시작점 (None이면 첫 그룹은 가장 점 많은 v로 시작)
    start_pos: Optional[Tuple[float, float]] = None


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def map_to_workspace(x: float, y: float, ws: Workspace, inp: InputSpec) -> Tuple[float, float]:
    """
    (x,y)를 로봇 작업영역 (rx,ry)로 맵핑.
    - inp.normalized_xy=True: x,y는 [0,1]로 가정
    - False: x,y는 픽셀, img_w/img_h 필요
    """
    if inp.normalized_xy:
        nx = x
        ny = y
    else:
        if inp.img_w <= 0 or inp.img_h <= 0:
            raise ValueError("img_w/img_h must be > 0 when normalized_xy=False")
        nx = x / inp.img_w
        ny = y / inp.img_h

    if ws.invert_y:
        ny = 1.0 - ny

    rx = ws.x_left + nx * (ws.x_right - ws.x_left)
    ry = ws.y_top  + ny * (ws.y_bottom - ws.y_top)

    if ws.clamp:
        rx = _clamp(rx, min(ws.x_left, ws.x_right), max(ws.x_left, ws.x_right))
        ry = _clamp(ry, min(ws.y_top, ws.y_bottom), max(ws.y_top, ws.y_bottom))

    return rx, ry


def order_points_nn(points: List[Tuple[float, float]],
                    start: Optional[Tuple[float, float]] = None,
                    cell_size: float = 5.0) -> List[Tuple[float, float]]:
    """
    pub_sm의 grid 기반 NN 정렬을 'float 좌표'로 일반화한 버전.
    """
    if not points:
        return []

    pts = points.copy()
    grid: Dict[Tuple[int, int], List[Tuple[float, float]]] = {}

    def cell_coord(p: Tuple[float, float]) -> Tuple[int, int]:
        return (int(math.floor(p[0] / cell_size)), int(math.floor(p[1] / cell_size)))

    for p in pts:
        c = cell_coord(p)
        grid.setdefault(c, []).append(p)

    def remove_point(p: Tuple[float, float]) -> None:
        c = cell_coord(p)
        bucket = grid.get(c)
        if not bucket:
            return
        # 동일 좌표 중복 가능하니 remove는 1회만
        bucket.remove(p)
        if not bucket:
            del grid[c]

    def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx*dx + dy*dy

    # 시작점 결정
    if start is None:
        current = pts[0]
    else:
        current = min(pts, key=lambda p: dist2(p, start))

    remove_point(current)
    pts.remove(current)

    ordered = [current]

    def find_nearby_point(cur: Tuple[float, float]) -> Tuple[float, float]:
        cx, cy = cell_coord(cur)
        radius = 0
        while True:
            # radius 링만 탐색
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if radius != 0 and (abs(dx) != radius and abs(dy) != radius):
                        continue
                    cell = (cx + dx, cy + dy)
                    bucket = grid.get(cell)
                    if bucket:
                        return min(bucket, key=lambda p: dist2(p, cur))
            radius += 1

    while pts:
        nxt = find_nearby_point(current)
        ordered.append(nxt)
        remove_point(nxt)
        pts.remove(nxt)
        current = nxt

    return ordered


def plan_stipple_path(dot_array,
                      ws: Workspace = Workspace(),
                      inp: InputSpec = InputSpec(),
                      cfg: PlannerConfig = PlannerConfig()
                      ) -> List[Tuple[float, float, int]]:
    """
    입력: dot_msgs/DotArray (dot_array.dots: [Dot(x,y,v), ...])
    출력: [(rx, ry, v), ...]  # v별 그룹핑 + 그룹 내부 NN + 그룹 간 순서 최적화 완료
    """

    # 1) v별 그룹핑 + 로봇좌표로 변환
    groups: Dict[int, List[Tuple[float, float]]] = {}
    for d in dot_array.dots:
        v = int(d.v)
        rx, ry = map_to_workspace(float(d.x), float(d.y), ws, inp)
        groups.setdefault(v, []).append((rx, ry))

    if not groups:
        return []

    # 2) 그룹 간 순서 결정 + 그룹 내부 NN
    def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx*dx + dy*dy

    remaining = set(groups.keys())
    path: List[Tuple[float, float, int]] = []

    # 시작점: 지정 없으면 "가장 점 많은 v"를 첫 그룹으로 (합리적 기본값)
    if cfg.start_pos is None:
        first_v = max(remaining, key=lambda vv: len(groups[vv]))
        current_pos: Optional[Tuple[float, float]] = None
        next_v = first_v
    else:
        current_pos = cfg.start_pos
        # start_pos에서 가장 가까운 그룹 고르기 (그룹의 어떤 점이든 최소거리 기준)
        next_v = min(
            remaining,
            key=lambda vv: min(dist2(p, current_pos) for p in groups[vv])
        )

    while remaining:
        v = next_v
        remaining.remove(v)

        pts = groups[v]

        # 그룹 내부: 현재 위치에 가장 가까운 점을 start로 잡아 NN 정렬
        if current_pos is None:
            ordered_pts = order_points_nn(pts, start=None, cell_size=cfg.cell_size)
        else:
            ordered_pts = order_points_nn(pts, start=current_pos, cell_size=cfg.cell_size)

        # path에 (rx,ry,v)로 append
        for (rx, ry) in ordered_pts:
            path.append((rx, ry, v))

        # 현재 위치 갱신 (그룹의 마지막 점)
        current_pos = ordered_pts[-1]

        if not remaining:
            break

        # 다음 그룹 선택: 현재 위치에서 "가장 가까운 그룹" (그룹 내 점 중 최소거리)
        next_v = min(
            remaining,
            key=lambda vv: min(dist2(p, current_pos) for p in groups[vv])
        )

    return path

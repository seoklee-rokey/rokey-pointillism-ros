# robot_mapper.py

# ===== 로봇 작업 영역 =====
X_LEFT = 320
Y_TOP = 0
X_RIGHT = 500
Y_BOTTOM = 120


# --------------------------------------------------
# Grid 기반 NN 정렬
def order_points_nn(points, cell_size=10):

    if len(points) == 0:
        return []

    pts = points.copy()
    grid = {}

    def cell_coord(p):
        return (int(p[0]) // cell_size, int(p[1]) // cell_size)

    for p in pts:
        c = cell_coord(p)
        grid.setdefault(c, []).append(p)

    def remove_point(p):
        c = cell_coord(p)
        grid[c].remove(p)
        if not grid[c]:
            del grid[c]

    current = pts[0]
    ordered = [current]
    remove_point(current)
    pts.remove(current)

    def find_nearest(cur):
        cx, cy = cell_coord(cur)
        radius = 0

        while True:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):

                    if abs(dx) != radius and abs(dy) != radius:
                        continue

                    cell = (cx + dx, cy + dy)

                    if cell in grid and grid[cell]:
                        return min(
                            grid[cell],
                            key=lambda p:
                            (p[0]-cur[0])**2 + (p[1]-cur[1])**2
                        )
            radius += 1


    while pts:
        next_pt = find_nearest(current)
        ordered.append(next_pt)
        remove_point(next_pt)
        pts.remove(next_pt)
        current = next_pt

    return ordered


# --------------------------------------------------
# 점묘화 → 로봇 좌표 변환
def convert_to_robot_coords(point_list, img_w, img_h):
    """
    point_list: [(x, y, v), ...]
    img_w, img_h: 이미지 크기
    return: [(rx, ry, v), ...]
    """

    if not point_list:
        return []

    work_w = X_RIGHT - X_LEFT
    work_h = Y_BOTTOM - Y_TOP

    img_ratio = img_w / img_h
    work_ratio = work_w / work_h

    if img_ratio > work_ratio:
        scale = work_w / img_w
    else:
        scale = work_h / img_h

    draw_w = img_w * scale
    draw_h = img_h * scale

    offset_x = X_LEFT + (work_w - draw_w) / 2
    offset_y = Y_TOP  + (work_h - draw_h) / 2

    robot_list = []

    # 색상 인덱스 기준 정렬
    sorted_points = sorted(point_list, key=lambda p: p[2])

    # 색상별 Grid NN 정렬
    from itertools import groupby

    for color_index, group in groupby(sorted_points, key=lambda p: p[2]):

        group_pts = [(p[0], p[1], p[2]) for p in group]

        # NN은 x,y 기준으로만
        ordered_xy = order_points_nn(
            [(p[0], p[1]) for p in group_pts]
        )

        for x, y in ordered_xy:

            rx = offset_x + x * scale
            ry = offset_y + y * scale

            robot_list.append((rx, ry, color_index))

    return robot_list

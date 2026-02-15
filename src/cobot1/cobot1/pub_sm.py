import cv2
import numpy as np
import random
import rclpy
from rclpy.node import Node

from dot_msgs.msg import Dot
from dot_msgs.msg import DotArray


# ===== 고정 파라미터 =====
CANNY_LOW = 80
CANNY_HIGH = 150

# ===== 로봇 작업 영역 =====
X_LEFT = 320
Y_TOP = 0
X_RIGHT = 500
Y_BOTTOM = 120

# ===== 팔레트 (1~18번 색상 인덱스 기준) =====
palette = [
    (0, 0, 0), (255, 255, 255), (255, 0, 0), (0, 255, 0),
    (0, 0, 255), (255, 255, 0), (255, 165, 0), (128, 0, 128),
    (255, 192, 203), (165, 42, 42), (128, 128, 128), (0, 255, 255),
    (0, 128, 0), (0, 0, 128), (255, 105, 180), (255, 140, 0),
    (173, 216, 230), (255, 250, 205)
]


# --------------------------------------------------
# 1️⃣ 이미지 리사이즈
def resize_keep_ratio(image, max_size):
    h, w = image.shape[:2]

    if max(h, w) <= max_size:
        return image

    scale = max_size / max(h, w)
    new_w = int(w * scale)
    new_h = int(h * scale)

    return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)


# --------------------------------------------------
# 2️⃣ 팔레트 최근접 색상 찾기
def nearest_color(r, g, b):
    min_dist = float('inf')
    nearest = palette[0]

    for pr, pg, pb in palette:
        dist = (int(r)-pr)**2 + (int(g)-pg)**2 + (int(b)-pb)**2
        if dist < min_dist:
            min_dist = dist
            nearest = (pr, pg, pb)

    return nearest


# --------------------------------------------------
# 3️⃣ Grid 기반 NN 정렬
def order_points_nn(points, start=None, cell_size=10):

    if len(points) == 0:
        return []

    pts = points.copy()
    grid = {}

    def cell_coord(p):
        return (p[0] // cell_size, p[1] // cell_size)

    for p in pts:
        c = cell_coord(p)
        if c not in grid:
            grid[c] = []
        grid[c].append(p)

    def remove_point(p):
        c = cell_coord(p)
        if c in grid and p in grid[c]:
            grid[c].remove(p)
            if len(grid[c]) == 0:
                del grid[c]

    current = pts[0] if start is None else min(
        pts, key=lambda p: (p[0]-start[0])**2 + (p[1]-start[1])**2)

    remove_point(current)
    pts.remove(current)

    ordered = [current]

    def find_nearby_point(cur):
        cx, cy = cell_coord(cur)
        radius = 0

        while True:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):

                    if abs(dx) != radius and abs(dy) != radius:
                        continue

                    cell = (cx + dx, cy + dy)

                    if cell in grid and grid[cell]:
                        return min(grid[cell], key=lambda p:
                                   (p[0]-cur[0])**2 + (p[1]-cur[1])**2)

            radius += 1

    while pts:
        next_pt = find_nearby_point(current)
        ordered.append(next_pt)
        remove_point(next_pt)
        pts.remove(next_pt)
        current = next_pt

    return ordered


# --------------------------------------------------
# 4️⃣ 점 생성
def generate_stipple_points(path, edge_prob,
                             inner_density, color_mode, max_size):

    image = cv2.imread(path)
    if image is None:
        raise ValueError("이미지 로드 실패")

    image = resize_keep_ratio(image, max_size)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)

    h, w = gray.shape
    color_points = {}

    for y in range(h):
        for x in range(w):

            draw_flag = False

            if edges[y, x] > 0:
                if random.random() < edge_prob:
                    draw_flag = True
            else:
                brightness = gray[y, x] / 255.0
                prob = (1 - brightness) * inner_density
                if random.random() < prob:
                    draw_flag = True

            if draw_flag:

                if color_mode == "bw":
                    rgb_color = (0, 0, 0)
                else:
                    b, g, r = image[y, x]
                    rgb_color = nearest_color(r, g, b)

                if rgb_color not in color_points:
                    color_points[rgb_color] = []

                color_points[rgb_color].append((x, y))

    ordered_color_points = {}
    for color, pts in color_points.items():
        ordered_color_points[color] = order_points_nn(pts)

    return ordered_color_points, w, h


# --------------------------------------------------
# 5️⃣ 로봇 좌표 변환 + 단일 리스트 생성
def convert_to_robot_list(points_dict, img_w, img_h):

    robot_list = []

    for color, pts in points_dict.items():
        for x, y in pts:
            rx = X_LEFT + (x / img_w) * (X_RIGHT - X_LEFT)
            ry = Y_TOP  + (y / img_h) * (Y_BOTTOM - Y_TOP)
            robot_list.append([rx, ry, color])

    return robot_list


# --------------------------------------------------
# 6️⃣ 색상 → 번호 (1~18)
def color_to_index(color):
    if color in palette:
        return palette.index(color) + 1
    return 1


# --------------------------------------------------
# 7️⃣ ROS2 퍼블리셔 노드
class DotPublisher(Node):

    def __init__(self, robot_points_list):
        super().__init__('dot_publisher')

        self.pub = self.create_publisher(DotArray, 'draw_this', 10)
        self.robot_points_list = robot_points_list

        self.publish_dots()

    def publish_dots(self):

        msg = DotArray()
        msg.dots = []

        for x, y, color in self.robot_points_list:

            d = Dot()
            d.x = float(x)
            d.y = float(y)
            d.v = int(color_to_index(color))

            msg.dots.append(d)

        self.pub.publish(msg)
        self.get_logger().info(f"Published {len(msg.dots)} dots")


# --------------------------------------------------
# 8️⃣ MAIN
def main():

    EDGE_PROB = 4.0
    INNER_DENSITY = 0.001
    COLOR_MODE = "color"
    MAX_SIZE = 400
    img_path = "/home/deepday/cobot_ws/src/cobot1/cobot1/1.jpeg"

    points_dict, img_w, img_h = generate_stipple_points(
        img_path,
        EDGE_PROB,
        INNER_DENSITY,
        COLOR_MODE,
        MAX_SIZE
    )

    robot_points_list = convert_to_robot_list(
        points_dict,
        img_w,
        img_h
    )

    rclpy.init()
    node = DotPublisher(robot_points_list)

    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

# --------------------------------------------------
# 9️⃣ 실행
if __name__ == "__main__":

    robot_points_list = main()
    print("총 점 개수:", len(robot_points_list))
    print(robot_points_list[:10])  # 일부 출력
    
    rclpy.init()
    node = DotPublisher(robot_points_list)

    rclpy.spin_once(node)   # 한 번만 퍼블리시
    node.destroy_node()
    rclpy.shutdown()
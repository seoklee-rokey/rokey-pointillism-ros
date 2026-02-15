#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import random
import rclpy
from rclpy.node import Node

from dot_msgs.msg import Dot
from dot_msgs.msg import DotArray

# ==============================
# Fixed parameters
# ==============================
CANNY_LOW_DEFAULT = 80
CANNY_HIGH_DEFAULT = 150

# Robot work area
X_LEFT = 250
Y_TOP = 0
X_RIGHT = 370
Y_BOTTOM = 120

# Palette (index 1~18)
palette = [
    (0, 0, 0), (255, 255, 255), (255, 0, 0), (0, 255, 0),
    (0, 0, 255), (255, 255, 0), (255, 165, 0), (128, 0, 128),
    (255, 192, 203), (165, 42, 42), (128, 128, 128), (0, 255, 255),
    (0, 128, 0), (0, 0, 128), (255, 105, 180), (255, 140, 0),
    (173, 216, 230), (255, 250, 205)
]

# ==============================
# Utils
# ==============================
def resize_keep_ratio(image, max_size: int):
    h, w = image.shape[:2]
    if max(h, w) <= max_size:
        return image, w, h

    scale = float(max_size) / float(max(h, w))
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return resized, new_w, new_h


def nearest_color(r, g, b):
    min_dist = float('inf')
    nearest = palette[0]
    for pr, pg, pb in palette:
        dist = (int(r) - pr) ** 2 + (int(g) - pg) ** 2 + (int(b) - pb) ** 2
        if dist < min_dist:
            min_dist = dist
            nearest = (pr, pg, pb)
    return nearest


def order_points_nn(points, start=None, cell_size=10):
    if not points:
        return []

    pts = points.copy()
    grid = {}

    def cell_coord(p):
        return (p[0] // cell_size, p[1] // cell_size)

    for p in pts:
        grid.setdefault(cell_coord(p), []).append(p)

    def remove_point(p):
        c = cell_coord(p)
        if c in grid and p in grid[c]:
            grid[c].remove(p)
            if not grid[c]:
                del grid[c]

    current = pts[0] if start is None else min(
        pts, key=lambda p: (p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2
    )

    remove_point(current)
    pts.remove(current)
    ordered = [current]

    def find_nearby_point(cur):
        cx, cy = cell_coord(cur)
        radius = 0
        while True:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    # perimeter only
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cell = (cx + dx, cy + dy)
                    if cell in grid and grid[cell]:
                        return min(
                            grid[cell],
                            key=lambda p: (p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2
                        )
            radius += 1

    while pts:
        nxt = find_nearby_point(current)
        ordered.append(nxt)
        remove_point(nxt)
        pts.remove(nxt)
        current = nxt

    return ordered


def color_to_index(color):
    if color in palette:
        return palette.index(color) + 1
    return 1


def convert_to_robot_list(points_dict, img_w, img_h):
    robot_list = []
    for color, pts in points_dict.items():
        for x, y in pts:
            rx = X_LEFT + (x / img_w) * (X_RIGHT - X_LEFT)
            ry = Y_TOP + (y / img_h) * (Y_BOTTOM - Y_TOP)
            robot_list.append([rx, ry, color])
    return robot_list


def generate_stipple_points(image_bgr,
                            edge_prob: float,
                            inner_density: float,
                            color_mode: str,
                            max_size: int,
                            canny_low: int,
                            canny_high: int,
                            nn_cell_size: int,
                            seed: int):
    """
    Returns: (ordered_color_points, resized_bgr, w, h)
    ordered_color_points: { (r,g,b): [(x,y), ...] }
    """
    random.seed(seed)

    img, w, h = resize_keep_ratio(image_bgr, max_size)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, canny_low, canny_high)

    color_points = {}

    # main sampling loop
    for y in range(h):
        for x in range(w):
            draw_flag = False

            if edges[y, x] > 0:
                if random.random() < edge_prob:
                    draw_flag = True
            else:
                brightness = gray[y, x] / 255.0
                prob = (1.0 - brightness) * inner_density
                if random.random() < prob:
                    draw_flag = True

            if draw_flag:
                if color_mode == "bw":
                    rgb_color = (0, 0, 0)
                else:
                    b, g, r = img[y, x]
                    rgb_color = nearest_color(r, g, b)

                color_points.setdefault(rgb_color, []).append((x, y))

    # order each color's points (optional but helps robot path)
    ordered_color_points = {}
    for color, pts in color_points.items():
        ordered_color_points[color] = order_points_nn(pts, cell_size=max(1, nn_cell_size))

    return ordered_color_points, img, w, h


def render_preview(points_dict, w, h, dot_radius: int, bg_rgb=(255, 255, 255)):
    # canvas is BGR
    canvas = np.full((h, w, 3), (bg_rgb[2], bg_rgb[1], bg_rgb[0]), dtype=np.uint8)
    total = 0
    for rgb, pts in points_dict.items():
        bgr = (rgb[2], rgb[1], rgb[0])
        for (x, y) in pts:
            cv2.circle(canvas, (int(x), int(y)), int(dot_radius), bgr, thickness=-1)
            total += 1
    return canvas, total


# ==============================
# ROS2 Publisher
# ==============================
class DotPublisher(Node):
    def __init__(self):
        super().__init__('dot_publisher_gui')
        self.pub = self.create_publisher(DotArray, 'draw_this', 10)

    def publish_robot_points(self, robot_points_list):
        msg = DotArray()
        msg.dots = []
        for x, y, color in robot_points_list:
            d = Dot()
            d.x = float(x)
            d.y = float(y)
            d.v = int(color_to_index(color))
            msg.dots.append(d)

        self.pub.publish(msg)
        self.get_logger().info(f"[PUBLISH] Published {len(msg.dots)} dots on 'draw_this'")


# ==============================
# OpenCV "GUI" (trackbars)
# ==============================
def main():
    # ---- Set your image path here ----
    img_path = "/home/deepday/cobot_ws/src/cobot1/cobot1/2.jpeg"

    original = cv2.imread(img_path)
    if original is None:
        raise ValueError(f"이미지 로드 실패: {img_path}")

    # Windows
    WIN_CTRL = "Controls"
    WIN_ORIG = "Original"
    WIN_PREV = "Preview"
    cv2.namedWindow(WIN_CTRL, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_ORIG, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_PREV, cv2.WINDOW_NORMAL)

    # Trackbars
    # edge_prob: 0~1000 => 0.000~1.000
    cv2.createTrackbar("EDGE_PROB x1000", WIN_CTRL, 200, 1000, lambda x: None)
    # inner_density: 0~300 => 0.000~0.300 (more than that explodes dots)
    cv2.createTrackbar("INNER_DENS x1000", WIN_CTRL, 50, 300, lambda x: None)
    # max_size: 80~800
    cv2.createTrackbar("MAX_SIZE", WIN_CTRL, 400, 800, lambda x: None)
    cv2.setTrackbarMin("MAX_SIZE", WIN_CTRL, 80)

    # dot thickness (preview only)
    cv2.createTrackbar("DOT_RADIUS", WIN_CTRL, 3, 10, lambda x: None)
    cv2.setTrackbarMin("DOT_RADIUS", WIN_CTRL, 1)

    # canny thresholds
    cv2.createTrackbar("CANNY_LOW", WIN_CTRL, CANNY_LOW_DEFAULT, 300, lambda x: None)
    cv2.createTrackbar("CANNY_HIGH", WIN_CTRL, CANNY_HIGH_DEFAULT, 400, lambda x: None)

    # NN cell size for ordering (bigger -> faster, less optimal path)
    cv2.createTrackbar("NN_CELL", WIN_CTRL, 10, 50, lambda x: None)
    cv2.setTrackbarMin("NN_CELL", WIN_CTRL, 3)

    # seed to stabilize preview
    cv2.createTrackbar("SEED", WIN_CTRL, 1, 999, lambda x: None)

    # color mode (0=bw, 1=color)
    cv2.createTrackbar("COLOR_MODE (0 BW / 1 COLOR)", WIN_CTRL, 1, 1, lambda x: None)

    # "button" style trackbar: set to 1 to publish once (auto resets to 0)
    cv2.createTrackbar("PUBLISH (set 1)", WIN_CTRL, 0, 1, lambda x: None)

    # ---- ROS init ----
    rclpy.init()
    node = DotPublisher()

    last_params = None
    last_compute_t = 0.0
    compute_interval = 0.15  # seconds (debounce)
    cached_robot_points = []
    cached_total = 0

    try:
        while True:
            # show original (resized for screen only)
            cv2.imshow(WIN_ORIG, original)

            # read params
            edge_prob = cv2.getTrackbarPos("EDGE_PROB x1000", WIN_CTRL) / 1000.0
            inner_density = cv2.getTrackbarPos("INNER_DENS x1000", WIN_CTRL) / 1000.0
            max_size = cv2.getTrackbarPos("MAX_SIZE", WIN_CTRL)
            dot_radius = cv2.getTrackbarPos("DOT_RADIUS", WIN_CTRL)
            canny_low = cv2.getTrackbarPos("CANNY_LOW", WIN_CTRL)
            canny_high = cv2.getTrackbarPos("CANNY_HIGH", WIN_CTRL)
            nn_cell = cv2.getTrackbarPos("NN_CELL", WIN_CTRL)
            seed = cv2.getTrackbarPos("SEED", WIN_CTRL)
            color_mode = "color" if cv2.getTrackbarPos("COLOR_MODE (0 BW / 1 COLOR)", WIN_CTRL) == 1 else "bw"
            publish_flag = cv2.getTrackbarPos("PUBLISH (set 1)", WIN_CTRL)

            # enforce canny relationship
            if canny_high < canny_low + 1:
                canny_high = canny_low + 1
                # reflect in UI
                if canny_high <= 400:
                    cv2.setTrackbarPos("CANNY_HIGH", WIN_CTRL, canny_high)

            params = (edge_prob, inner_density, max_size, dot_radius, canny_low, canny_high, nn_cell, seed, color_mode)

            now = time.time()
            # recompute if params changed + debounce time passed
            if (params != last_params) and (now - last_compute_t >= compute_interval):
                last_compute_t = now
                last_params = params

                # generate + preview
                points_dict, resized_img, w, h = generate_stipple_points(
                    original,
                    edge_prob=edge_prob,
                    inner_density=inner_density,
                    color_mode=color_mode,
                    max_size=max_size,
                    canny_low=canny_low,
                    canny_high=canny_high,
                    nn_cell_size=nn_cell,
                    seed=seed
                )

                preview_img, total = render_preview(points_dict, w, h, dot_radius=dot_radius)

                # overlay info
                info = f"dots={total} | edge_prob={edge_prob:.3f} inner={inner_density:.3f} max={max_size} mode={color_mode}"
                cv2.putText(preview_img, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 2, cv2.LINE_AA)

                cv2.imshow(WIN_PREV, preview_img)

                # cache publish data
                cached_robot_points = convert_to_robot_list(points_dict, w, h)
                cached_total = total

            # publish if user toggled "button"
            if publish_flag == 1:
                if cached_robot_points:
                    node.publish_robot_points(cached_robot_points)
                else:
                    node.get_logger().warn("[PUBLISH] No points to publish.")
                # reset the "button"
                cv2.setTrackbarPos("PUBLISH (set 1)", WIN_CTRL, 0)

            # let ROS pump once (non-blocking)
            rclpy.spin_once(node, timeout_sec=0.0)

            # key handling
            k = cv2.waitKey(30) & 0xFF
            if k == 27 or k == ord('q'):  # ESC or q
                break

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

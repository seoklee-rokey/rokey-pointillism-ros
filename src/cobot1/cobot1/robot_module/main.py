# test_robot_mapper.py

from stipple_robot_planner import convert_to_robot_coords


def main():

    # 액션 통신으로 받았다고 가정한 점묘화 데이터
    stipple_points = [
        (10, 10, 1),
        (50, 30, 2),
        (20, 15, 1),
        (60, 40, 2),
        (25, 18, 1),
        (65, 42, 2),
    ]

    img_w = 200
    img_h = 140

    robot_points = convert_to_robot_coords(
        stipple_points,
        img_w,
        img_h
    )

    print("입력 점 개수:", len(stipple_points))
    print("출력 점 개수:", len(robot_points))
    print("변환 결과 일부:")
    print(robot_points[:10])


if __name__ == "__main__":
    main()

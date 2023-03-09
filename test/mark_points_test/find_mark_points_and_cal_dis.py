import argparse
import numpy as np
import math

# 输入3个文件路径参数
# 点序号-坐标.csv文件：[N][point_index, px, py]
# 打点序号-时间戳.csv文件：[N][point_index, ms]
# 轨迹结果.csv文件：[N][ms, user_phone, x, y]
if __name__ == '__main__':
    # 接收控制台参数
    # 接收3个参数
    parser = argparse.ArgumentParser()
    parser.add_argument("points", type=str)
    parser.add_argument("marks", type=str)
    parser.add_argument("xy", type=str)
    args = parser.parse_args()

    points_file = args.points
    marks_file = args.marks
    xy_file = args.xy

    # 读取3个文件
    points_data = np.loadtxt(points_file, delimiter=',')
    marks_data = np.loadtxt(marks_file, delimiter=',')
    xy_data = np.loadtxt(xy_file, delimiter=',')

    # 根据点序号找出打点的每个点的真值坐标
    result_list = []
    for mark in marks_data:
        for point in points_data:
            if mark[0] == point[0]:
                # 打点时间，点序号，点坐标x,y
                result_list.append([mark[1], mark[0], point[1], point[2]])

    # 在轨迹结果里根据时间戳找出对应的预测坐标
    xy_index = 0
    xy_len = len(xy_data)
    for i in range(0, len(result_list)):
        mark_time = result_list[i][0]
        while xy_index < xy_len and xy_data[xy_index][0] < mark_time:
            xy_index += 1
        if xy_index < xy_len:
            result_list[i].append(xy_data[xy_index][2])
            result_list[i].append(xy_data[xy_index][3])
            result_list[i].append(math.hypot(result_list[i][2]-result_list[i][4], result_list[i][3]-result_list[i][5]))

    # 打印结果
    for result in result_list:
        if len(result) == 7:
            # point 1: (x, y)  position: (x, y)  distance: x m \n
            print("point ", result[1], ':(', result[2], ',', result[3], ')  position:(', result[4], ',', result[5], ')  distance:', result[6], ' m\n')
import os

import numpy as np
import math
import mag_and_other_tools.paint_tools as PT


# 计算两点的距离
def two_points_dis(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# 计算两直线斜率，k2相对k1逆时针旋转的度数
def two_slope_angle_off(v1, v2):
    # 方向向量
    x1, y1 = v1
    x2, y2 = v2
    dot = x1 * x2 + y1 * y2
    det = x1 * y2 - y1 * x2
    theta = np.arctan2(det, dot)
    theta = theta if theta > 0 else 2 * np.pi + theta
    return theta


# 坐标变换函数，论文公式4.7。求x1, y1关于transfer的偏导矩阵，论文公式4.13
# 输入：变换向量transfer=[_x, _y, _angle(弧度，绕PDR坐标系的(0, 0)逆时针)]，被变换坐标PDR(x0,y0)
# 输出：转换后的坐标 x1, y1，偏导矩阵 grad_xy(2*3)
# NOTE: t_angle是弧度！不是°。 degrees(x) : 将弧度转化为角度  radians(x) : 将角度转化为弧度。
def transfer_axis(transfer, x0, y0):
    _x = transfer[0]
    _y = transfer[1]
    _angle = transfer[2]
    m_angle = np.array([[math.cos(_angle), -math.sin(_angle)],
                        [math.sin(_angle), math.cos(_angle)]])
    m_move = np.array([[_x],
                       [_y]])
    m_xy = np.array([[x0],
                     [y0]])
    ans = np.dot(m_angle, m_xy) + m_move
    return ans[0][0], ans[1][0]


# --------------------建库文件--------------------------------
# 根据打点坐标调整PDR轨迹
# 0.输入3个文件，原始IMU文件、通过AI_PDR程序获得的.npy文件、打点文件
# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/mag map build/2/TEST_2022-07-28-145643_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/mag map build/2/TEST_2022-07-28-145643_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/mag map build/2/TEST_2022-07-28-145643_points.csv"
# ]

# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/mag map build/3/TEST_2022-07-28-145932_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/mag map build/3/TEST_2022-07-28-145932_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room  test/mag map build/3/TEST_2022-07-28-145932_points.csv"
# ]
#
# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/mag map build/4/TEST_2022-07-28-150211_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/mag map build/4/TEST_2022-07-28-150211_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/mag map build/4/TEST_2022-07-28-150211_points.csv"
# ]

# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/mag map build/5/TEST_2022-07-28-150518_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/mag map build/5/TEST_2022-07-28-150518_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/mag map build/5/TEST_2022-07-28-150518_points.csv"
# ]

# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/mag map build/6/TEST_2022-07-28-150805_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/mag map build/6/TEST_2022-07-28-150805_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/mag map build/6/TEST_2022-07-28-150805_points.csv"
# ]

# -------------------测试文件--------------------------------
# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/position test/1/TEST_2022-07-28-152101_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/position test/1/TEST_2022-07-28-152101_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/position test/1/TEST_2022-07-28-152101_points.csv"
# ]

# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/position test/2/TEST_2022-07-28-152234_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/position test/2/TEST_2022-07-28-152234_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/position test/2/TEST_2022-07-28-152234_points.csv"
# ]

# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/position test/3/TEST_2022-07-28-152352_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/position test/3/TEST_2022-07-28-152352_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/position test/3/TEST_2022-07-28-152352_points.csv"
# ]
#
# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/position test/4/TEST_2022-07-28-152525_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/position test/4/TEST_2022-07-28-152525_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/position test/4/TEST_2022-07-28-152525_points.csv"
# ]
#
# path_list = [
#     # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
#     "./data/server room test/position test/5/TEST_2022-07-28-152643_sensors.csv",
#     # PDR坐标文件：x,y
#     "./data/server room test/position test/5/TEST_2022-07-28-152643_sensors.csv.npy",
#     # 打点文件：时间戳，x,y
#     "./data/server room test/position test/5/TEST_2022-07-28-152643_points.csv"
# ]
#
path_list = [
    # IMU文件：时间戳，加速度，陀螺仪，磁力计，四元数
    "./data/server room test/position test/6/TEST_2022-07-28-152749_sensors.csv",
    # PDR坐标文件：x,y
    "./data/server room test/position test/6/TEST_2022-07-28-152749_sensors.csv.npy",
    # 打点文件：时间戳，x,y
    "./data/server room test/position test/6/TEST_2022-07-28-152749_points.csv"
]

dir_path = os.path.dirname(path_list[0])
file_save_path = dir_path + "/marked_pdr_xy.csv"

START = 0  # imu数据切掉的部分
ABANDON_REMNANT = True  # 舍弃最后（最后打的点之后）多余的数据

# 1.先根据时间戳将打点坐标和PDR坐标进行映射:
imu_time_arr = np.loadtxt(path_list[0], delimiter=",")[START:, 0]
pdr_xy_arr = np.load(path_list[1]) / 1000
PT.paint_xy_list([pdr_xy_arr], ["PDR"])
#   每个PDR_xy对应10个IMU数据，第i个PDR_xy的时间戳是第10*i个IMU文件的时间戳
pdr_time_arr = np.empty(shape=len(pdr_xy_arr), dtype=float)
for i in range(0, len(pdr_xy_arr)):
    pdr_time_arr[i] = imu_time_arr[10 * (i + 1)]
#   根据时间戳，找到打点坐标对应的pdr坐标
mark_xy_arr = np.loadtxt(path_list[2], delimiter=",")
PT.paint_xy_list([mark_xy_arr[:, 1:3]], ["mark points"])
mark_pdr_index_map = []
pdr_index = 0
for mark_index in range(0, len(mark_xy_arr)):
    mark_time = mark_xy_arr[mark_index, 0]
    while pdr_index < len(pdr_time_arr) - 1 and pdr_time_arr[pdr_index] < mark_time:
        pdr_index += 1
    mark_pdr_index_map.append([mark_index, pdr_index])

# 2. 将相邻mark points之间的pdr points 缩放、旋转 到mark points
mark_pdr_index_map = np.array(mark_pdr_index_map)
new_pdr_xy_list = []
start_transfer = None
for i in range(1, len(mark_pdr_index_map)):
    # 目标点
    start_mark_xy = mark_xy_arr[mark_pdr_index_map[i - 1, 0], 1:3].copy()
    end_mark_xy = mark_xy_arr[mark_pdr_index_map[i, 0], 1:3].copy()
    print("start_mark_xy:", start_mark_xy, "end_mark_xy", end_mark_xy)
    # pdr始末点
    start_pdr_xy = pdr_xy_arr[mark_pdr_index_map[i - 1, 1], :].copy()
    end_pdr_xy = pdr_xy_arr[mark_pdr_index_map[i, 1], :].copy()
    print("start_pdr_index:", mark_pdr_index_map[i - 1, 1], "end_pdr_index:", mark_pdr_index_map[i, 1])
    print("start_pdr_xy:", start_pdr_xy, "end_pdr_xy", end_pdr_xy)
    # pdr子段（如果是最后一段，则判断是否要包括多余的）
    sub_pdr_xy = pdr_xy_arr[mark_pdr_index_map[i - 1, 1]:mark_pdr_index_map[i, 1] + 1, :].copy() if i != len(
        mark_pdr_index_map) - 1 or ABANDON_REMNANT else pdr_xy_arr[mark_pdr_index_map[i - 1, 1]:, :].copy()
    # pdr -> mark 的距离缩放倍数
    zoom_k = two_points_dis(start_mark_xy, end_mark_xy) / two_points_dis(start_pdr_xy, end_pdr_xy)
    print("zoom_k:", zoom_k)
    # 将该段pdr所有的相邻坐标都进行缩放，不会改变始末点直线的斜率
    print("缩放前dis:", two_points_dis(sub_pdr_xy[0], sub_pdr_xy[len(sub_pdr_xy) - 1]))
    print("预期新dis:", two_points_dis(sub_pdr_xy[0], sub_pdr_xy[len(sub_pdr_xy) - 1]) * zoom_k)
    for j in range(1, len(sub_pdr_xy)):
        # 平移到原点
        move_x = -sub_pdr_xy[j - 1, 0]
        move_y = -sub_pdr_xy[j - 1, 1]
        sub_pdr_xy[:, 0] += move_x
        sub_pdr_xy[:, 1] += move_y
        # 缩放距离
        old_xy = sub_pdr_xy[j]
        new_xy = old_xy * zoom_k
        # 将 j 及其之后的所有点平移
        move_x_1 = new_xy[0] - old_xy[0]
        move_y_1 = new_xy[1] - old_xy[1]
        sub_pdr_xy[j:, 0] += move_x_1
        sub_pdr_xy[j:, 1] += move_y_1
    print("缩放后dis:", two_points_dis(sub_pdr_xy[0], sub_pdr_xy[len(sub_pdr_xy) - 1]))

    # 将这段放大后的pdr旋转至与mark_xy对齐
    # 计算两直线的 始末点向量 夹角，方向向量 = end_point - start_point
    vector_pdr = end_pdr_xy[0] - start_pdr_xy[0], end_pdr_xy[1] - start_pdr_xy[1]
    vector_mark = end_mark_xy[0] - start_mark_xy[0], end_mark_xy[1] - start_mark_xy[1]
    angle_off = two_slope_angle_off(vector_pdr, vector_mark)
    # 坐标转换
    # 先平移到原点，再计算变换向量（因为是先绕原点逆时针旋转再平移）
    move_x_2 = sub_pdr_xy[0, 0]
    move_y_2 = sub_pdr_xy[0, 1]
    sub_pdr_xy[:, 0] -= move_x_2
    sub_pdr_xy[:, 1] -= move_y_2
    transfer = [start_mark_xy[0], start_mark_xy[1], angle_off]
    start_transfer = transfer.copy() if i == 1 else start_transfer
    print("transfer", transfer[0], ',', transfer[1], ',', math.degrees(transfer[2]), '\n')
    for xy in sub_pdr_xy:
        x, y = transfer_axis(transfer, xy[0], xy[1])
        new_pdr_xy_list.append([x, y])

# 输出、保存结果
PT.paint_xy_list([np.array(new_pdr_xy_list)], ["newPDR"])
PT.paint_xy_list([mark_xy_arr[:, 1:3], np.array(new_pdr_xy_list), pdr_xy_arr],
                 ["markPoints", "newPDR", "rawPDR"],
                 [0, 16, -15, 15],
                 save_file=dir_path + "/marked_pdr_xy_contrast.png")
np.savetxt(file_save_path, new_pdr_xy_list, delimiter=",")

pdr_xy_start_index = mark_pdr_index_map[0, 1]
pdr_xy_end_index = mark_pdr_index_map[len(mark_pdr_index_map) - 1, 1]
pdr_xy_change_inf = [[start_transfer[0], start_transfer[1], start_transfer[2], pdr_xy_start_index, pdr_xy_end_index]]
np.savetxt(dir_path + '/pdr_xy_change_inf.csv', pdr_xy_change_inf, delimiter=',')
print('Start Transfer to (0,0): {0} \nPDR xy Start Index = {1} \n\t\tEnd index = {2}'.format(
    [start_transfer[0], start_transfer[1], math.degrees(start_transfer[2])], pdr_xy_start_index, pdr_xy_end_index
))

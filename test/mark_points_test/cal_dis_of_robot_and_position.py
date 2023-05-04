import numpy as np
import math
import argparse


# 计算机器人坐标和定位轨迹坐标之间的：点对点距离、平均距离
# 输入机器人 txt 文件，定位轨迹结果 csv 文件 {time, id, x, y}

# 从给的txt文件中读出，指定属性并保存到列表中[ms, robot_x, robot_y]
# 读出的秒和纳秒，要组合成毫秒
def read_robot_result(robot_txt_file):
    data_list = []

    # 0:找 "secs:", 1:找"nsecs", 2:找"position:", 3:找"x:", 4:找"y:"
    find_what = 0
    file_handler = open(robot_txt_file, "r")
    while True:
        line = file_handler.readline()
        if not line:
            break
        #
        if find_what == 0 and "secs:" in line:
            secs_str = line.split(':')[1]
            find_what = 1
        else:
            if find_what == 1 and "nsecs:" in line:
                nsecs_str = line.split(':')[1]
                find_what = 2
                # 将秒和纳秒的高三位合并
                msecs_str = str(int(secs_str)) + str(int(nsecs_str))[0:3]
            else:
                if find_what == 2 and "position:" in line:
                    find_what = 3
                else:
                    if find_what == 3 and "x:" in line:
                        px_str = line.split(':')[1]
                        find_what = 4
                    else:
                        if find_what == 4 and "y:" in line:
                            py_str = line.split(':')[1]
                            find_what = 0
                            # 将结果保存到list中
                            data_list.append([float(msecs_str), float(px_str), float(py_str)])

    return data_list


# 输入：
#   robot_data_list [robot_time, x, y]
#   position_arr [position_time, id, x, y]
#   offset：以定位结果的时间为准，所以 position_time = robot_time + offset，offset = position_time - robot_time
# 输出：
#   由于应该是 机器人的位置 输出频率更高，所以按position的xy在机器人里面找
#   但还是为了防止意外情况，允许机器人时间戳重复使用！
# 计算单点距离、平均距离、小于1m占比，print结果，并且保持print的内容到指定txt文件
def cal_print_save_dis(robot_data_list, position_arr, offset, inf_save_file):
    result_msg_file = open(inf_save_file, "w", encoding='GBK')

    robot_index = 0
    robot_len = len(robot_data_list)
    time_xy_list = []  # [time, robot_x, robot_y, pos_x, pos_y]

    for pos in position_arr:
        robot_time = float(pos[0]) - offset
        while robot_index < robot_len and float(robot_data_list[robot_index][0]) < robot_time:
            robot_index += 1

        if robot_index < robot_len:
            # 将二者坐标加入到list
            time_xy_list.append([pos[0],
                                 robot_data_list[robot_index][1],
                                 robot_data_list[robot_index][2],
                                 float(pos[2]),
                                 float(pos[3])])
        else:
            break

    # 计算结果、打印结果、统计结果、保存结果
    one_meter_less_num = 0
    two_meter_less_num = 0
    dis_sum = 0
    for line in time_xy_list:
        time = line[0]
        rx = line[1]
        ry = line[2]
        px = line[3]
        py = line[4]
        dis = math.hypot(rx - px, ry - py)
        dis_sum += dis
        if dis <= 1:
            one_meter_less_num += 1
        else:
            if dis <= 2:
                two_meter_less_num += 1
        # 打印结果
        print("{0}: robot({1:.2f}, {2:.2f}), position({3:.2f}, {4:.2f}). Distance: {5:.2f} m".format(
            int(time), rx, ry, px, py, dis
        ))
        # 保存结果
        print("{0}: robot({1:.2f}, {2:.2f}), position({3:.2f}, {4:.2f}). Distance: {5:.2f} m".format(
            int(time), rx, ry, px, py, dis
        ), file=result_msg_file)

    # 统计结果
    print("\nMean Distance: {0:.2f} m".format((dis_sum / len(time_xy_list))))
    print("Distance less 1m: {0:.2f} %".format((one_meter_less_num / len(time_xy_list)) * 100))
    print("Distance less 2m: {0:.2f} %".format((two_meter_less_num / len(time_xy_list)) * 100))

    print("\nMean Distance: {0:.2f} m".format((dis_sum / len(time_xy_list))), file=result_msg_file)
    print("Distance less 1m: {0:.2f} %".format((one_meter_less_num / len(time_xy_list)) * 100), file=result_msg_file)
    print("Distance less 2m: {0:.2f} %".format((two_meter_less_num / len(time_xy_list)) * 100), file=result_msg_file)


if __name__ == '__main__':
    # 调用
    parser = argparse.ArgumentParser()
    parser.add_argument("robot")
    parser.add_argument("pos")
    parser.add_argument("offset")
    parser.add_argument("save")
    args = parser.parse_args()

    robot_txt_file = args.robot
    pos_csv_file = args.pos
    offset = float(args.offset)
    inf_save_txt_file = args.save

    robot_data_list = read_robot_result(robot_txt_file)
    pos_data_arr = np.loadtxt(pos_csv_file, delimiter=",")

    cal_print_save_dis(robot_data_list, pos_data_arr, offset, inf_save_txt_file)

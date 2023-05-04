import numpy as np


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


# 将手机IMU文件、机器人文件进行对齐、
# 以手机IMU时间为准，所以 imu_time = robot_time + offset，offset = imu_time - robot_time
# 获得[N][ms, px, py, imu_data_except_time]
def sync_robot_pos_and_imu_data(robot_txt_file, imu_csv_file, offset):
    # 调用read_robot_result()获取[ms, robot_x, robot_y]
    robot_data_list = read_robot_result(robot_txt_file)
    imu_data_arr = np.loadtxt(imu_csv_file, delimiter=',')

    # 机器人位置频率更低，将手机imu数据合并到机器人那边，以保证文件不会过大!
    imu_index = 0
    imu_len = len(imu_data_arr)
    sync_data_list = []

    for robot_line in robot_data_list:
        imu_time = robot_line[0] + offset
        # 在imu数据中找到对应时间
        while imu_index < imu_len and float(imu_data_arr[imu_index][0]) < imu_time:
            imu_index += 1

        if imu_index < imu_len:
            # 将robot_line和imu_data_list[imu_index][1:]合并
            temp_data = []
            for d1 in robot_line:
                temp_data.append(d1)
            for d2 in imu_data_arr[imu_index][1:]:
                temp_data.append(float(d2))
            sync_data_list.append(temp_data)
        else:
            break

    return sync_data_list


# robot_test_txt = "D:\pythonProjects\MagPdrServer_forPackage\\test\\robot_data_test\\robot_result.txt"
# imu_test_csv = "D:/pythonProjects/MagPdrServer_forPackage/test/robot_data_test/imu_fake_test_data.csv"

# robot_data = read_robot_result(robot_test_txt)
# print("robot data:")
# print(len(robot_data), ',', len(robot_data[0]))
# np.savetxt("D:\pythonProjects\MagPdrServer_forPackage\\test\\robot_data_test\\robot_xy.csv", robot_data, delimiter=',')
# for d in robot_data:
#     print(d)

# sync_data = sync_robot_pos_and_imu_data(robot_test_txt, imu_test_csv, 0)
# print(len(sync_data), ',', len(sync_data[0]))
# print(len(sync_data[0]))
# for d in sync_data:
#     print(d)
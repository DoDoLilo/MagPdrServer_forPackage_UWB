# 该脚本将ilocator.csv文件中除了第一列时间戳的数据列（共7列），对齐、拼接到imu.csv文件的所有列的后面
# 我们取其 （倒数第5，倒数4列）的数据作为GT_xy
import csv
import sys


def _postposcess_slam(slam_raw_path: str):
    res = []
    f = open(slam_raw_path)
    csv_reader = csv.reader(f)
    for row in csv_reader:
        row[0] = int(row[0]) / 1000000
        row_string = str(row).replace('[', '').replace(']', '').replace("'", '')
        temp = row_string.split(',')
        res.append(temp)
    f.close()
    return res


def _postposcess_imu(imu_raw_path: str):
    res = []
    f = open(imu_raw_path)
    csv_reader = csv.reader(f)
    for row in csv_reader:
        row[0] = int(row[0])
        row_string = str(row).replace(' ', '').replace('[', '').replace(']', '').replace("'", '')
        temp = row_string.split(',')
        res.append(temp)
    f.close()
    return res


def _slam_and_imu_sync(slam_post_list, imu_post_list, offset=0):
    p = 0
    res = []
    for row_imu in imu_post_list:
        time1 = float(row_imu[0])
        # if p >= len(slam_post_list):
        #         break
        imu_row = slam_post_list[p]
        time2 = float(imu_row[0]) - offset
        while time2 <= time1:
            # print("time1:", time1, "time2:", time2)
            p += 1
            # if p >= len(slam_post_list):
            #     break
            imu_row = slam_post_list[p]
            time2 = float(imu_row[0]) - offset
        p = p - 1
        tp = str(slam_post_list[p][1:])
        t_str = str(str(row_imu) + ',' + tp).replace(' ', '').replace("'", '').replace('[', '').replace(']', '')
        # print('time 1: %f, time 2: %f imu time - slam time %f' %(time1, time2,time1-time2))
        res.append(t_str.split(','))
    return res


def timestamp_sync(data, freq: int):
    time = 0
    offset = 1 / freq
    res = []
    for i in range(0, len(data)):
        tt = data[i].split(',')
        t = float(tt[0])
        while (time < t):
            res.append(
                str(time) + ',' + str(tt[1:]).replace(' ', '').replace("'", '').replace('[', '').replace(']', ''))
            time += offset
    return res


# 将slam_csv_path文件中除了第一列时间戳的数据列（共7列），对齐、拼接到imu_csv_path文件的所有列的后面，直接返回数据结果不保存文件
# 我们取其 （倒数第5，倒数4列）的数据作为GT_xy
def sync_slam_pos_and_imu_data(slam_csv_path: str, imu_csv_path: str, offset=0):
    slam_post = _postposcess_slam(slam_csv_path)
    imu_post = _postposcess_imu(imu_csv_path)
    res = _slam_and_imu_sync(slam_post, imu_post, offset)
    # res是个list
    return res


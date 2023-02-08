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
    print("offet,", offset)
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
        res.append(t_str)
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


# 将slam_csv_path文件中除了第一列时间戳的数据列（共7列），对齐、拼接到imu_csv_path文件的所有列的后面，并保存至文件out_file_path
# 我们取其 （倒数第5，倒数4列）的数据作为GT_xy
def sync_slam_pos_and_imu_data_and_save(slam_csv_path: str, imu_csv_path: str, out_file_path: str = 'defalut.csv', offset=0):
    f = open(out_file_path, 'w+')
    print('start post sync processing...')
    slam_post = _postposcess_slam(slam_csv_path)
    imu_post = _postposcess_imu(imu_csv_path)
    print('temp processing done...')
    print('start syncing...')
    res = _slam_and_imu_sync(slam_post, imu_post, offset)
    for row in res:
        f.write(row + '\n')
    print('done. point data saved in %s' % (out_file_path))


# 将slam_csv_path文件中除了第一列时间戳的数据列（共7列），对齐、拼接到imu_csv_path文件的所有列的后面，直接返回数据结果不保存文件
# 我们取其 （倒数第5，倒数4列）的数据作为GT_xy
def sync_slam_pos_and_imu_data(slam_csv_path: str, imu_csv_path: str, offset=0):
    slam_post = _postposcess_slam(slam_csv_path)
    imu_post = _postposcess_imu(imu_csv_path)
    res = _slam_and_imu_sync(slam_post, imu_post, offset)
    # res是个list
    return res


if __name__ == "__main__":
    slam_path = r'D:\OneDrive - whu.edu.cn\MagMap test data\InfCenter server room\iLocator\20220812_101552_\_2022-08-12-10-16-02.bag.pbstream_x1y1x2y2.csv'
    imu_path = r'D:\OneDrive - whu.edu.cn\MagMap test data\InfCenter server room\phone IMU\IMU-812-9-189.79622112889115 Pixel 6.csv'
    out_path = r'D:\OneDrive - whu.edu.cn\MagMap test data\InfCenter server room\synced_with_ilocator\IMU-812-9-189.79622112889115 Pixel 6_sync.csv'
    # offset(ms) 1237
    offset = -865456
    freq = 200
    sync_slam_pos_and_imu_data_and_save(slam_path, imu_path, out_path, freq, offset)

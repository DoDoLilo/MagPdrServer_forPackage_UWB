import os
import mag_and_other_tools.mag_mapping_tools as MMT
import numpy as np

# 坐标系位移（平移，单位：米）
MOVE_X = 0.
MOVE_Y = 0.
# 地图坐标系大小 0-MAP_SIZE_X ，0-MAP_SIZE_Y（单位：米）
MAP_SIZE_X = 20.
MAP_SIZE_Y = 15.
# 地图地磁块大小
BLOCK_SIZE = 0.25
# 低通滤波的程度，值越大滤波越强。整型，无单位。
EMD_FILTER_LEVEL = 3
# 内插半径
INTER_RADIUS = 1
# 内插迭代次数上限，目前未使用。目前方案是内插满后再根据DELETE_LEVEL进行删除
# INTER_TIME_THR = 2
# pdr滑动窗口距离
PDR_IMU_ALIGN_SIZE = 10
# 是否删除多余内插块
DELETE_EXTRA_BLOCKS = True
# 删除多余内插块的程度，越大删除的内插范围越大，可以为负值。
DELETE_LEVEL = 1

# imu.csv, marked_pdr_xy.csv
file_paths_build_map = [
    ["./data/server room test/mag map build/1/TEST_2022-07-28-145322_sensors.csv",
     "./data/server room test/mag map build/1/marked_pdr_xy.csv"],

    ["./data/server room test/mag map build/2/TEST_2022-07-28-145643_sensors.csv",
     "./data/server room test/mag map build/2/marked_pdr_xy.csv"],

    ["./data/server room test/mag map build/3/TEST_2022-07-28-145932_sensors.csv",
     "./data/server room test/mag map build/3/marked_pdr_xy.csv"],

    ["./data/server room test/mag map build/4/TEST_2022-07-28-150211_sensors.csv",
     "./data/server room test/mag map build/4/marked_pdr_xy.csv"],

    ["./data/server room test/mag map build/5/TEST_2022-07-28-150518_sensors.csv",
     "./data/server room test/mag map build/5/marked_pdr_xy.csv"],

    ["./data/server room test/mag map build/6/TEST_2022-07-28-150805_sensors.csv",
     "./data/server room test/mag map build/6/marked_pdr_xy.csv"]
]

# 构建文件夹
save_dir_name = 'map_F'
for file_path in file_paths_build_map:
    save_dir_name += os.path.basename(os.path.dirname(file_path[0])) + '_'
save_dir_name += "B_{0}".format(BLOCK_SIZE)
save_dir_path = 'data/server room test/mag_map/' + save_dir_name
save_dir_path += '_full' if not DELETE_EXTRA_BLOCKS else '_deleted'
if not os.path.exists(save_dir_path):
    os.mkdir(save_dir_path)

mag_map = MMT.build_map_by_files_and_marked_pdr_xy(
    file_paths=file_paths_build_map,
    map_size_x=MAP_SIZE_X, map_size_y=MAP_SIZE_Y,
    # time_thr=INTER_TIME_THR,
    radius=INTER_RADIUS, block_size=BLOCK_SIZE,
    delete_extra_blocks=DELETE_EXTRA_BLOCKS,
    delete_level=DELETE_LEVEL,
    lowpass_filter_level=EMD_FILTER_LEVEL,
    pdr_imu_align_size=PDR_IMU_ALIGN_SIZE,
    fig_save_dir=save_dir_path
)

# 保存建库文件
np.savetxt(save_dir_path + '/mv_qiu_2d.csv', mag_map[:, :, 0], delimiter=',')
np.savetxt(save_dir_path + '/mh_qiu_2d.csv', mag_map[:, :, 1], delimiter=',')
print("Save files to:", save_dir_path)

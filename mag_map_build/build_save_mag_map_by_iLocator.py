# 使用iLocator建库，使用的是对齐后的IMU和ilocator xy数据（原点固定的xy），不需要跑PDR的结果。
# TODO 为了封装，则需要将 对齐功能 包括进来：
#   1、所以接收的参数是1个文件路径，指向一个保存了[N][ilocator.csv, imu_file.csv, offset]的csv文件
#       + 1个iLocator offset参数
#       + 1个建库输出目录地址
#       + 1个json配置文件路径
#   2、将imu_file.csv, ilocator.csv输入至sync_points.py方法中，获得对齐后的 imu_ilocator_sync.csv文件
#   3、读取imu_ilocator_sync.csv文件，调用方法建库
#   4、对应需要修改的是MMT.build_map_by_files_and_ilocator_xy方法中的列位置，让其能取到正确的imu\gt_xy数据
#   5、通过配置文件，读取出所有常量参数MOVE_XY\MAP_SIZE_XY ... ...
import mag_and_other_tools.mag_mapping_tools as MMT
import mag_and_other_tools.file_tools as FT
import mag_map_build.sync_point as SP
import numpy as np
import os
import argparse
from mag_and_other_tools.config_tools import SystemConfigurations
from mag_and_other_tools.config_tools import ConfigTypes


if __name__ == '__main__':
    # 接收3个参数
    parser = argparse.ArgumentParser()
    parser.add_argument("files")
    parser.add_argument("out_dir", type=str)
    parser.add_argument("json", type=str)
    args = parser.parse_args()

    file_list_path = args.files  # 指向保存了[N][ilocator.csv, imu_file.csv, offset]的csv文件
    map_save_dir = args.out_dir  # 指纹库输出路径
    config_json_file = args.json  # 配置文件路径

    # 初始化文件数组[N][ilocator.csv, imu_file.csv, offset]
    init_files_succeed = True
    gt_imu_offset_list = None  # [N][ilocator.csv, imu_file.csv, offset]
    if FT.file_name_end_with(file_list_path, FT.FileEnd.CSV) and FT.file_is_exist(file_list_path):
        # 读出file_list文件中的内容[N][ilocator.csv, imu_file.csv, offset]
        gt_imu_offset_list = np.loadtxt(file_list_path, delimiter=",")
        # 检查这些文件是否都合规
        for imu_gt_offset in gt_imu_offset_list:
            if len(gt_imu_offset_list) != 3:
                init_files_succeed = False
                print("File error: wrong shape of imu_gt_offset_list. Check the [imu.csv, gt.csv, offset] format.")
                break
            # 检查所有imu\gt.csv文件后缀、是否存在
            if not (FT.file_name_end_with(imu_gt_offset[0], FT.FileEnd.CSV) and FT.file_is_exist(imu_gt_offset[0]) and
                    FT.file_name_end_with(imu_gt_offset[1], FT.FileEnd.CSV) and FT.file_is_exist(imu_gt_offset[1])):
                print("File error: some file is not end with csv or is not exist.")
                init_files_succeed = False
                break

    if init_files_succeed:
        # TODO 文件名没有问题（不代表文件内容没问题，接着执行再说吧），正常执行建库代码，并保存到指定路径
        configurations = SystemConfigurations(config_json_file, ConfigTypes.MAP_BUILD)
        if not configurations.init_succeed:
            print("Read config json file failed! Please stop starting the program and checking the config json file!")
            for inf in configurations.failed_inf:
                print(inf)
        else:
            # 读取配置文件初始化成功，从配置文件中读出上述常量参数
            # 地图坐标系大小 0-MAP_SIZE_X ，0-MAP_SIZE_Y（m）
            MAP_SIZE_X = configurations.MapSizeX
            MAP_SIZE_Y = configurations.MapSizeY
            # 坐标系平移参数（m）
            MOVE_X = configurations.MoveX
            MOVE_Y = configurations.MoveY
            # 地图地磁块大小
            BLOCK_SIZE = configurations.BlockSize
            # 低通滤波的程度，值越大滤波越强。整型，无单位。
            EMD_FILTER_LEVEL = configurations.EmdFilterLevel
            # 内插半径
            INTER_RADIUS = configurations.InterRadius
            # 是否删除多余内插块
            DELETE_EXTRA_BLOCKS = configurations.DeleteExtraBlocks
            # 删除多余内插块的程度，越大删除的内插范围越大，可以为负值。
            DELETE_LEVEL = configurations.DeleteLevel

            # 正式开始建库：
            # 1.调用sync_points.py对imu_gt_offset_list中的多个imu\ilocator文件进行offset对齐合并
            #   得到原本的多个_sync.csv文件对应的数据结果，直接在这里做合并，转为ndarray
            imu_gt_sync_data = []
            for gt_imu_offset in gt_imu_offset_list:
                sync_data = SP.sync_slam_pos_and_imu_data(gt_imu_offset[0], gt_imu_offset[1], gt_imu_offset[2])
                for row in sync_data:
                    imu_gt_sync_data.append(row)
            imu_gt_sync_data = np.array(imu_gt_sync_data)

            # 2.将多个文件对齐、合并后的一个ndarray进行建库
            mag_map = MMT.build_map_by_imu_gt_sync_ndarray(imu_gt_sync_data, MOVE_X, MOVE_Y, MAP_SIZE_X, MAP_SIZE_Y,
                                                           inter_radius=INTER_RADIUS, block_size=BLOCK_SIZE,
                                                           delete_extra_blocks=DELETE_EXTRA_BLOCKS,
                                                           delete_level=DELETE_LEVEL,
                                                           lowpass_filter_level=EMD_FILTER_LEVEL,
                                                           fig_save_dir=map_save_dir
                                                           )

            # 3.将返回的指纹库保存到指定路径下
            np.savetxt(map_save_dir + '/mv.csv', mag_map[:, :, 0], delimiter=',')
            np.savetxt(map_save_dir + '/mh.csv', mag_map[:, :, 1], delimiter=',')
            print("Save files to:", map_save_dir)

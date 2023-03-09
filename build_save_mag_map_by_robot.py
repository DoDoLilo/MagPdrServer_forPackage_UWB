import mag_and_other_tools.mag_mapping_tools as MMT
import mag_and_other_tools.file_tools as FT
import mag_map_build.sync_robot_and_imu_data as SRI
import numpy as np
import argparse
from mag_and_other_tools.config_tools import SystemConfigurations
from mag_and_other_tools.config_tools import ConfigTypes


# 使用最新的use_line_num条文件进行建库
# 输入：
# use_line_num：使用最新的几条
# line_num：文件记录总条数
#   return 保存了有效的行下标的set
def get_the_use_lines(use_line_num, line_num):
    final_line_set = set()

    # 要求数量<=0，则使用默认情况，最新的2个
    if use_line_num <= 0:
        use_line_num = 2

    # 要求数量比已有的记录多，则使用全部
    if use_line_num > line_num:
        use_line_num = line_num

    # 添加记录
    for i in range(line_num - use_line_num, line_num):
        final_line_set.add(i)

    return final_line_set


# 将iLocator修改为robot
if __name__ == '__main__':
    # 接收4个参数
    parser = argparse.ArgumentParser()
    parser.add_argument("files")
    parser.add_argument("out_dir", type=str)
    parser.add_argument("json", type=str)
    parser.add_argument("use", type=str)
    args = parser.parse_args()

    file_list_path = args.files  # 指向保存了[N][robot_result.txt, imu_file.csv, offset]的csv文件
    map_save_dir = args.out_dir  # 输出的指纹库保存目录
    config_json_file = args.json  # 配置文件路径
    use_line_num = int(args.use)  # 选取清单中的哪些行进行建库

    # 初始化文件数组[N][robot_result.txt, imu_file.csv, offset]
    init_files_succeed = True
    gt_imu_offset_arr = None  # [N][robot_result.txt, imu_file.csv, offset]
    if FT.file_name_end_with(file_list_path, FT.FileEnd.CSV.value) and FT.file_is_exist(file_list_path):
        # 读出file_list文件中的内容[N][robot_result.txt, imu_file.csv, offset]
        gt_imu_offset_arr = np.loadtxt(file_list_path, delimiter=",", dtype=str)
        if gt_imu_offset_arr.ndim == 1:
            # 只有一行记录，读出来的数组变成1维了，需要扩展为二维
            gt_imu_offset_arr = gt_imu_offset_arr[np.newaxis, :]
        # 如果清单文件中不存在可用记录（len=0），则认为“建库失败”。
        line_num = len(gt_imu_offset_arr)
        if line_num == 0:
            init_files_succeed = False
            print("List File error: there is no record in the file.")
        # 检查这些文件是否都合规
        for gt_imu_offset in gt_imu_offset_arr:
            if len(gt_imu_offset) != 3:
                init_files_succeed = False
                print("List File error: wrong shape of imu_gt_offset_list. Check the [robot.txt, imu.csv, offset] format.")
                break
            # 检查所有imu\gt.csv文件后缀、是否存在
            if not (FT.file_name_end_with(gt_imu_offset[0], FT.FileEnd.TXT.value) and FT.file_is_exist(
                    gt_imu_offset[0]) and
                    FT.file_name_end_with(gt_imu_offset[1], FT.FileEnd.CSV.value) and FT.file_is_exist(
                        gt_imu_offset[1])):
                print("List File error: some file is not end with csv or is not exist.")
                init_files_succeed = False
                break
    else:
        print("List File not exist!")
        init_files_succeed = False

    if init_files_succeed:
        # 文件名没有问题（不代表文件内容没问题，接着执行再说吧），正常执行建库代码，并保存到指定路径
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
            # 0.先获得清单文件中的哪些行是用于建库
            use_line_set = get_the_use_lines(use_line_num, line_num)

            # 1.调用sync_points.py对imu_gt_offset_list中的多个robot_result.txt/imu文件进行offset对齐合并
            #   得到原本的多个_sync.csv文件对应的数据结果，直接在这里做合并，转为ndarray
            gt_imu_sync_data = []
            for line_index in range(0, line_num):
                if line_index in use_line_set:
                    print(gt_imu_offset_arr[line_index])
                    sync_data = SRI.sync_robot_pos_and_imu_data(gt_imu_offset_arr[line_index][0],
                                                                gt_imu_offset_arr[line_index][1],
                                                                float(gt_imu_offset_arr[line_index][2]))
                    for row in sync_data:
                        gt_imu_sync_data.append(row)
            gt_imu_sync_data = np.array(gt_imu_sync_data, dtype=float)

            # 2.将多个文件对齐、合并后的一个ndarray进行建库
            mag_map = MMT.build_map_by_robot_gt_imu_sync_ndarray(gt_imu_sync_data, MOVE_X, MOVE_Y, MAP_SIZE_X,
                                                                 MAP_SIZE_Y,
                                                                 inter_radius=INTER_RADIUS, block_size=BLOCK_SIZE,
                                                                 delete_extra_blocks=DELETE_EXTRA_BLOCKS,
                                                                 delete_level=DELETE_LEVEL,
                                                                 lowpass_filter_level=EMD_FILTER_LEVEL,
                                                                 fig_save_dir=map_save_dir
                                                                 )

            # 3.将返回的指纹库保存到指定路径下
            np.savetxt(map_save_dir + '/mv.csv', mag_map[:, :, 0], delimiter=',')
            np.savetxt(map_save_dir + '/mh.csv', mag_map[:, :, 1], delimiter=',')
            print("Save files to dir:" + map_save_dir)

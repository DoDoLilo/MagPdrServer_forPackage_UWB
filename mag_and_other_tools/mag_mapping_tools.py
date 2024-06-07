import math
from enum import Enum
import numpy as np
from PyEMD import EMD
from scipy.spatial.transform import Rotation
import mag_and_other_tools.paint_tools as PT
import queue


# 读csv文件所有列，返回指定列：输入：文件路径，返回：指定列数组numpy.ndarray
def get_data_from_csv(path, row_start=-1, row_end=-1):
    if row_start == row_end == -1:
        return np.loadtxt(path, delimiter=",")
    if row_start < 0:
        row_start = 0
    return np.loadtxt(path, delimiter=",", usecols=range(row_start, row_end + 1))


# cut_off: 舍弃的高频信号分量数量，该值越大，滤掉的高频信号越多
def lowpass_emd(data_magnitude, cut_off):
    emd = EMD()
    imfs = emd(data_magnitude)
    data_filtered = np.sum(imfs[cut_off:, :], axis=0)
    return data_filtered


# q为四元数（game rotation vector)
# 输入为序列，直接并行处理，不再是一个一个处理
# 返回垂直磁强、水平磁强和总磁强
def get_2d_mag_qiu(ori_quat, mag):
    ori_R = Rotation.from_quat(ori_quat)
    glob_mag = np.einsum("tip,tp->ti", ori_R.as_matrix(), mag)
    mag_v = np.abs(glob_mag[:, 2:3])  # 垂直分量 = z轴结果
    mag_h = np.linalg.norm(glob_mag[:, 0:2], axis=1, keepdims=True)  # 水平分量 = x, y的合
    mag_total = np.linalg.norm(glob_mag, axis=1, keepdims=True)  # 总量 = x,y,z的合
    # return np.concatenate([mag_v, mag_h, mag_total], axis=1)
    return np.concatenate([mag_v, mag_h], axis=1)


# 内插填补
# 输入：mag_map[i][j][2]:栅格化、坐标正数化，后的三维数组，保存栅格i,j的平均磁强mv,mh ，=-1表示无效
#      radius：填补所用的半径范围（m）
#      block_size: 单个栅格的大小（m）
# 输出：空块、填充成功的块、失败的块
def interpolation_to_fill(mag_map, radius=1.0, block_size=0.3):
    # 0、使用copy数组以避免嵌套填补
    copy_mag_map = np.copy(mag_map)
    mask_list = []
    # 1、通过radius和block_size计算出斜向、非斜向的最远块数(向下取整)
    slant_most = int(radius / math.sqrt(2 * block_size ** 2))
    not_slant_most = int(radius / block_size)
    # 2、对无效的mag_map[i][j],在这个范围内去找8个方向的有效栅格块，保存有效块的candidates[][3]{mv,mh,distance}
    slant_directions = [[-1, -1], [+1, +1], [-1, +1], [+1, -1]]
    not_slant_directions = [[-1, 0], [+1, 0], [0, -1], [0, +1]]
    empty_list = []
    succeed_list = []
    failed_list = []
    for i in range(0, len(mag_map)):
        for j in range(0, len(mag_map[0])):
            if mag_map[i][j][0] == -1:
                empty_list.append([i, j])
                # candidates保存候选项的 mv, mh, distance
                candidates = []
                # 4对方向，按对同步搜索
                for t in 0, 1:
                    # 斜向2对
                    find1 = search_by_direction_distance(copy_mag_map, i, j, slant_directions[2 * t], slant_most)
                    find2 = search_by_direction_distance(copy_mag_map, i, j, slant_directions[2 * t + 1], slant_most)
                    if find1[0] != -1 and find2[0] != -1:
                        candidates.append(find1)
                        candidates.append(find2)
                    # 水平向2对
                    find1 = search_by_direction_distance(copy_mag_map, i, j, not_slant_directions[2 * t],
                                                         not_slant_most)
                    find2 = search_by_direction_distance(copy_mag_map, i, j, not_slant_directions[2 * t + 1],
                                                         not_slant_most)
                    if find1[0] != -1 and find2[0] != -1:
                        candidates.append(find1)
                        candidates.append(find2)
                # 3、根据distance进行加权求出当前块的mv',mh'，替换原来的mag_map值
                if len(candidates) == 0:
                    failed_list.append([i, j])
                else:
                    succeed_list.append([i, j])
                    mag_map[i][j][0], mag_map[i][j][1] = cal_weighted_average(candidates)

    return [empty_list, succeed_list, failed_list]


# 返回找到的：磁强mv,mh，距离distance
# 未找到则返回-1,-1,0
def search_by_direction_distance(mag_map, i, j, direction, most_far):
    for distance in range(0, most_far):
        i = i + direction[0]
        j = j + direction[1]
        if -1 < i < len(mag_map) and -1 < j < len(mag_map[0]):
            if mag_map[i][j][0] != -1 and mag_map[i][j][1] != -1:
                return [mag_map[i][j][0], mag_map[i][j][1], distance + 1]
        else:
            break
    return [-1, -1, 0]


# 计算加权平均并返回
def cal_weighted_average(candidates):
    weight_all = 0
    mv_ave = 0
    mh_ave = 0
    for c in candidates:
        # 距离distance越大，权重越小
        weight_all += 1 / c[2]
    for c in candidates:
        w = (1 / c[2]) / weight_all
        mv_ave += w * c[0]
        mh_ave += w * c[1]
    return mv_ave, mh_ave


# 数组arr_mv_mh栅格化 block_size=0.3(m)
# 输入：arr_mv_mh[N][2] ， x_y_GT轨迹[N][2]，地图范围，块大小
# 输出：rast_mv_mh[x][y][mv][mh]
# 思路：①将机房固定分块；②落于同一块中的x_y_GT的对应mv_mh进行平均
# 实现：arr_1保存平均结果， arr_2保存落入当前块的点个数 n
def build_rast_mv_mh(arr_mv_mh, arr_xy_gt, map_size_x, map_size_y, block_size):
    # 先根据地图、块大小，计算块的个数，得到数组的长度。向上取整
    shape = [math.ceil(map_size_x / block_size), math.ceil(map_size_y / block_size), 2]
    rast_mv_mh = np.empty(shape, dtype=float)
    # 不用考虑平均前求sum时候的溢出情况? sys.float_info.max（1.7976931348623157e + 308）
    # 创建的rast_mv_mh全置 -1
    for i in range(0, shape[0]):
        for j in range(0, shape[1]):
            rast_mv_mh[i][j][0] = -1
            rast_mv_mh[i][j][1] = -1

    # 创建用来记录个数的数字，blocks_num
    blocks_num = np.zeros([shape[0], shape[1]], dtype=int)
    # 遍历arr_mv_mh\arr_xy_gt计算块总和
    for i in range(0, len(arr_xy_gt)):
        block_x = int(arr_xy_gt[i][0] // block_size)
        block_y = int(arr_xy_gt[i][1] // block_size)
        rast_mv_mh[block_x][block_y][0] += arr_mv_mh[i][0]
        rast_mv_mh[block_x][block_y][1] += arr_mv_mh[i][1]
        blocks_num[block_x][block_y] += 1
    # 平均
    for i in range(0, shape[0]):
        for j in range(0, shape[1]):
            if blocks_num[i][j] != 0:
                rast_mv_mh[i][j] /= blocks_num[i][j]
                # rast_mv_mh[i][j][0] /= blocks_num[i][j]
                # rast_mv_mh[i][j][1] /= blocks_num[i][j]

    return rast_mv_mh


# 数组mag_map坐标-坐标正数化：GT坐标 --> 全正数坐标
# 思路：每幅图的xy是固定一样的1P2，所以将坐标原点移动至左下角即可
# 实现：x,y直接加上一个固定整数，GT坐标边界{x轴-7m ~ 1m，y轴-8m ~ 5m} +7,+8 -->
#                             全正数坐标{x轴 0m ~ 8m，y轴 0m ~ 13m}
# 数组mag_map坐标-坐标反正数化：全正数坐标 --> GT坐标
# 思路：正数化的逆操作即可
def change_axis(arr_x_y_gt, move_x, move_y):
    for i in range(0, len(arr_x_y_gt)):
        arr_x_y_gt[i][0] += move_x
        arr_x_y_gt[i][1] += move_y
    return


# 手动挑出ilocator图片质量好的csv文件进行建库·
# 输入：原始csv文件路径file_paths，各种建库参数
# 输出：栅格化的地磁双分量数组
# 实现：①根据路径读取数组；②将多个文件的{地磁、方向、xyGT}连接为一个数组后进行建库。
def build_map_by_files_and_ilocator_xy(file_paths, move_x, move_y, map_size_x, map_size_y,
                                       time_thr=-1, radius=1, block_size=0.3,
                                       delete_extra_blocks=False, delete_level=0,
                                       lowpass_filter_level=3,
                                       fig_save_dir=None):
    if len(file_paths) == 0:
        return None

    data_all = get_data_from_csv(file_paths[0])
    data_mag = data_all[:, 21:24]
    data_quat = data_all[:, 7:11]  # GAME_ROTATION_VECTOR 未经磁场矫正的旋转向量（四元数）
    data_x_y = data_all[:, np.shape(data_all)[1] - 5:np.shape(data_all)[1] - 3]

    for i in range(1, len(file_paths)):
        data_all = get_data_from_csv(file_paths[i])
        data_mag = np.vstack((data_mag, data_all[:, 21:24]))
        data_quat = np.vstack((data_quat, data_all[:, 7:11]))
        data_x_y = np.vstack((data_x_y, data_all[:, np.shape(data_all)[1] - 5:np.shape(data_all)[1] - 3]))

    # 地磁转为垂直、水平分量mv\mh
    arr_mv_mh = get_2d_mag_qiu(data_quat, data_mag)
    # emd滤波，太慢了，不过是建库阶段，无所谓
    mv_filtered_emd = lowpass_emd(arr_mv_mh[:, 0], lowpass_filter_level)
    mh_filtered_emd = lowpass_emd(arr_mv_mh[:, 1], lowpass_filter_level)
    arr_mv_mh = np.vstack((mv_filtered_emd, mh_filtered_emd)).transpose()
    # 栅格化
    change_axis(data_x_y, move_x, move_y)
    rast_mv_mh = build_rast_mv_mh(arr_mv_mh, data_x_y, map_size_x, map_size_y, block_size)
    PT.paint_heat_map(rast_mv_mh, save_dir=fig_save_dir + '/no_inter' if fig_save_dir is not None else None)
    # 内插填补，绘制结果
    rast_mv_mh_before_inter_fill = rast_mv_mh.copy()
    inter_fill_completely(rast_mv_mh, time_thr, radius, block_size)
    if delete_extra_blocks:
        delete_far_blocks(rast_mv_mh_before_inter_fill, rast_mv_mh, radius, block_size, delete_level)
    PT.paint_heat_map(rast_mv_mh, save_dir=fig_save_dir + '/intered' if fig_save_dir is not None else None)
    return rast_mv_mh


# 使用对齐、合并后的imu datas\ground truth xy 数组数据进行地磁指纹库建立
#   输入：[N][,,] 与各种建库参数
#   输出：栅格化的地磁双分量数组
def build_map_by_imu_gt_sync_ndarray(imu_gt_sync_data,
                                     move_x, move_y, map_size_x, map_size_y,
                                     time_thr=-1, inter_radius=1, block_size=0.3,
                                     delete_extra_blocks=False, delete_level=0,
                                     lowpass_filter_level=3,
                                     fig_save_dir=None):
    cols = np.shape(imu_gt_sync_data)[1]
    data_mag = imu_gt_sync_data[:, 21: 24]
    data_quat = imu_gt_sync_data[:, 7: 11]
    data_x_y = imu_gt_sync_data[:, cols - 5: cols - 3]

    # 地磁转为垂直、水平分量mv\mh
    arr_mv_mh = get_2d_mag_qiu(data_quat, data_mag)
    # emd滤波，太慢了，不过是建库阶段，无所谓
    mv_filtered_emd = lowpass_emd(arr_mv_mh[:, 0], lowpass_filter_level)
    mh_filtered_emd = lowpass_emd(arr_mv_mh[:, 1], lowpass_filter_level)
    arr_mv_mh = np.vstack((mv_filtered_emd, mh_filtered_emd)).transpose()
    # 栅格化
    change_axis(data_x_y, move_x, move_y)
    rast_mv_mh = build_rast_mv_mh(arr_mv_mh, data_x_y, map_size_x, map_size_y, block_size)
    PT.paint_heat_map(rast_mv_mh, save_dir=fig_save_dir + '/no_inter' if fig_save_dir is not None else None)
    # 内插填补，绘制结果
    rast_mv_mh_before_inter_fill = rast_mv_mh.copy()
    inter_fill_completely(rast_mv_mh, time_thr, inter_radius, block_size)
    if delete_extra_blocks:
        delete_far_blocks(rast_mv_mh_before_inter_fill, rast_mv_mh, inter_radius, block_size, delete_level)
    PT.paint_heat_map(rast_mv_mh, save_dir=fig_save_dir + '/intered' if fig_save_dir is not None else None)
    return rast_mv_mh


def build_map_by_robot_gt_imu_sync_ndarray(gt_imu_sync_data,
                                           move_x, move_y, map_size_x, map_size_y,
                                           time_thr=-1, inter_radius=1, block_size=0.3,
                                           delete_extra_blocks=False, delete_level=0,
                                           lowpass_filter_level=3,
                                           fig_save_dir=None):
    data_mag = gt_imu_sync_data[:, 23: 26]
    data_quat = gt_imu_sync_data[:, 9: 13]
    data_x_y = gt_imu_sync_data[:, 1: 3]

    # 地磁转为垂直、水平分量mv\mh
    arr_mv_mh = get_2d_mag_qiu(data_quat, data_mag)
    # emd滤波，太慢了，不过是建库阶段，无所谓
    mv_filtered_emd = lowpass_emd(arr_mv_mh[:, 0], lowpass_filter_level)
    mh_filtered_emd = lowpass_emd(arr_mv_mh[:, 1], lowpass_filter_level)
    arr_mv_mh = np.vstack((mv_filtered_emd, mh_filtered_emd)).transpose()
    # 栅格化
    change_axis(data_x_y, move_x, move_y)
    rast_mv_mh = build_rast_mv_mh(arr_mv_mh, data_x_y, map_size_x, map_size_y, block_size)
    PT.paint_heat_map(rast_mv_mh, save_dir=fig_save_dir + '/no_inter' if fig_save_dir is not None else None)
    # 内插填补，绘制结果
    rast_mv_mh_before_inter_fill = rast_mv_mh.copy()
    inter_fill_completely(rast_mv_mh, time_thr, inter_radius, block_size)
    if delete_extra_blocks:
        delete_far_blocks(rast_mv_mh_before_inter_fill, rast_mv_mh, inter_radius, block_size, delete_level)
    PT.paint_heat_map(rast_mv_mh, save_dir=fig_save_dir + '/intered' if fig_save_dir is not None else None)
    return rast_mv_mh


# 循环调用内插填补
# 输入:栅格化rast_mv_mh，循环次数上限time_threshold(-1则循环直到不存在新增块),填补半径radius,块大小block_size
# 输出：内插填补的次数 num
def inter_fill_completely(rast_mv_mh, time_thr=-1, radius=1, block_size=0.3):
    num = 0
    while True:
        num += 1
        if len(interpolation_to_fill(rast_mv_mh, radius, block_size)[1]) == 0 or num == time_thr:
            break
    return num


# 输入：内插前的栅格化数组rast_raw，内插后数组rast_inter，内插半径，块大小，删除程度(允许<0)
# delete_level: =0的时候，保留内插半径圆范围内的块（所以=0的时候也可能进行删除），>0的时候删除的更多，<0的时候保留范围会扩大！
# 输出：保留原始位置一个半径内的内插值后的数组
# 实现：1、copy rast_raw；2、遍历rast_raw，将要保留的位置在copy中置 1；3、修改rast_inter
def delete_far_blocks(rast_mv_mh_raw, rast_mv_mh_inter, radius, block_size, delete_level):
    # 1
    if len(rast_mv_mh_raw) == 0:
        return
    copy_rast_raw = np.copy(rast_mv_mh_raw)
    far_most = int(radius / math.sqrt(2 * block_size ** 2)) - delete_level
    if far_most < 0:
        far_most = 0
    # 2
    len_1 = len(rast_mv_mh_raw)
    len_2 = len(rast_mv_mh_raw[0])
    for i in range(0, len_1):
        for j in range(0, len_2):
            if rast_mv_mh_raw[i][j][0] != -1 or rast_mv_mh_raw[i][j][1] != -1:
                for b_i in range(i - far_most, i + far_most + 1):
                    for b_j in range(j - far_most, j + far_most + 1):
                        if -1 < b_i < len_1 and -1 < b_j < len_2:
                            copy_rast_raw[b_i][b_j][0] = 1
    # 3
    for i in range(0, len_1):
        for j in range(0, len_2):
            if copy_rast_raw[i][j][0] == -1:
                rast_mv_mh_inter[i][j][0] = -1
                rast_mv_mh_inter[i][j][1] = -1
    return


# --------------------------匹配阶段的算法----------------------------------------------------
# 采样缓冲池_非实时（也使用累积距离）包括稀疏采样处理
# 实现：1、计算mv,mh; 2、计算PDR累计距离
# 输入：缓冲池长度buffer_dis，下采样距离down_sip_dis，采集的测试序列[N][quat[4], mag[3], [x,y]]
#   正式测试匹配时输入的data_xy应该是PDR_x_y，而不是iLocator_xy，而且输出的是单条匹配序列
#  NOTE:此时data_mag\quat还需与PDR_x_y对齐（不需要精确对齐？）后再输入（且PDR_xy为20帧/s和iLocator/手机 200帧/s不同），
#  输出变为[PDR_x, PDR_y, aligned_mmv, aligned_mmh]
# 输出：多条匹配序列[?][M][x,y, mv, mh]，注意不要转成np.array，序列长度M不一样
def samples_buffer(buffer_dis, down_sip_dis, data_quat, data_mag, data_xy,
                   do_filter=False, lowpass_filter_level=3):
    # 计算mv,mh分量,得到[N][mv, mh]
    arr_mv_mh = get_2d_mag_qiu(data_quat, data_mag)
    # 滤波：对匹配序列中的地磁进行滤波，在下采样之前滤波，下采样之后太短了不能滤波？
    if do_filter:
        mv_filtered_emd = lowpass_emd(arr_mv_mh[:, 0], lowpass_filter_level)
        mh_filtered_emd = lowpass_emd(arr_mv_mh[:, 1], lowpass_filter_level)
        arr_mv_mh = np.vstack((mv_filtered_emd, mh_filtered_emd)).transpose()
    # for遍历data_xy，计算距离，达到down_sip_dis/2记录 i_mid，达到 down_sip_dis记录 i_end并计算down_sampling
    i_start = 0
    i_mid = -1
    dis_sum_temp = 0
    dis_sum_all = 0
    dis_mid = down_sip_dis / 2
    match_seq_list = []
    match_seq = []
    for i in range(1, len(data_xy)):
        dis_sum_temp += math.hypot(data_xy[i][0] - data_xy[i - 1][0], data_xy[i][1] - data_xy[i - 1][1])
        if dis_sum_temp >= down_sip_dis:
            # 当PDR仅两帧之间距离直接> down_sip_dis，则会导致连续进入该flow，使得 i_mid=-1
            if i_mid == -1:
                i_mid = i_start
            match_seq.append(down_sampling(i_start, i_mid, i, data_xy, arr_mv_mh))
            i_start = i
            i_mid = -1
            dis_sum_all += dis_sum_temp
            # dis_sum_temp -= down_sip_dis 这样写虽然很真实，但会导致dis_sum_temp与i_mid_xy含义不一致，
            # 之前的会影响后续的，而且在dis_sum_all+= dis_sum_temp的时候会重复加上之前未清0的距离
            dis_sum_temp = 0
        else:
            if i_mid == -1 and dis_sum_temp >= dis_mid:
                i_mid = i

        if dis_sum_all >= buffer_dis:
            # dis_sum_all -= buffer_dis 这样写，之前的会影响后续的
            dis_sum_all = 0
            match_seq_list.append(match_seq.copy())
            match_seq.clear()

    match_seq_list.append(match_seq)
    # Don't change to numpy array at this time, because the len(match_seq) is different
    return match_seq_list


# 对实时采集到的原始磁场序列进行重采样（以空间距离为尺度）
# NOTE: 重采样前是要先进行mvh计算的，因为计算mvh和对应的quat_ori相关，而角度不能平均，显然不能平均后再计算mvh。
# 实现：按累积距离下采样：假设为0.3m，则0.15m处x,y的磁强 = 0.0 ~ 0.3m的磁强平均（坐标不能平均）
# 输入：下采样窗口下标start,mid,end，采集的匹配序列[x, y, mv, mh]
# 输出：稀疏的 位置-磁场 序列[pdr_x, pdr_y, mmv, mmh, pdr_xy_index]
def down_sampling(i_start, i_mid, i_end, data_xy, arr_mv_mh):
    mmv = np.mean(arr_mv_mh[i_start:i_end, 0])
    mmh = np.mean(arr_mv_mh[i_start:i_end, 1])
    return data_xy[i_mid][0], data_xy[i_mid][1], mmv, mmh, i_mid


# 对建立的方格指纹库进行双线性插值法+偏微分
# 输入：处理完毕的地磁指纹库（栅格化、内插）mag_map_vh[x][y][mv, mh]，需要获取地磁插值、梯度的坐标 x,y ，block_size
# 输出：np.array[线性插值后的磁强[l_mv, l_mh], 2对梯度分量[[[[mv/x , mv/y]], [[mh/x, mh/y]]]] :[2][1×2]
# 实现：1、当前点 x,y得到4候选块下标 : [x//bs, x//bs +1] * [y//bs, y//bs +1]
#      2、候选块下标对应原始坐标到x,y的距离: x-x0=x%bs, x1-x=bs-x%bs, x1-x0=bs; y-y0=y%bs, y1-y=bs-y%bs, y1-y0=bs
# NOTE: 越界处理：1、若x//bs or y//bs越界，则直接返回-1，-1表示越界错误；2、若+1后越界，则用x//bs替代 x//bs +1
#    未越界但磁强不存在：1、x//bs or y//bs未越界但磁强不存在：返回-1，-1；2、+1后未越界但不存在，则用x//bs替代 x//bs +1
def get_linear_map_mvh_with_grad(mag_map, x, y, block_size):
    b_x = int(x // block_size)
    b_y = int(y // block_size)
    num_b_x = len(mag_map)
    num_b_y = 0 if num_b_x == 0 else len(mag_map[0])

    if -1 < b_x < num_b_x and -1 < b_y < num_b_y:
        m_p00 = mag_map[b_x][b_y]
        if m_p00[0] == -1 or m_p00[1] == -1:
            return np.array([-1, -1]), np.array([[[-1, -1]], [[-1, -1]]])

        m_p01 = mag_map[b_x][b_y + 1] if b_y + 1 < num_b_y and mag_map[b_x][b_y + 1][0] != -1 else m_p00
        m_p10 = mag_map[b_x + 1][b_y] if b_x + 1 < num_b_x and mag_map[b_x + 1][b_y][0] != -1 else m_p00
        m_p11 = mag_map[b_x + 1][b_y + 1] if b_x + 1 < num_b_x and b_y + 1 < num_b_y and mag_map[b_x + 1][b_y + 1][
            0] != -1 else m_p00

        x_x0 = x % block_size
        x1_x = block_size - x_x0
        x1_x0 = block_size
        y_y0 = y % block_size
        y1_y = block_size - y_y0
        y1_y0 = block_size
        m_pxy = y_y0 * (x_x0 * m_p11 + x1_x * m_p01) / (x1_x0 * y1_y0) + y1_y * (x_x0 * m_p10 + x1_x * m_p00) / (
                x1_x0 * y1_y0)
        grad_px = y_y0 * (m_p11 - m_p01) / (x1_x0 * y1_y0) + y1_y * (m_p10 - m_p00) / (x1_x0 * y1_y0)
        grad_py = x_x0 * (m_p11 - m_p10) / (x1_x0 * y1_y0) + x1_x * (m_p01 - m_p00) / (x1_x0 * y1_y0)

        return m_pxy, np.array([[[grad_px[0], grad_py[0]]], [[grad_px[1], grad_py[1]]]])

    print("Error: Out of Whole Map!")
    return np.array([-1, -1]), np.array([[[-1, -1]], [[-1, -1]]])


# 对get_linear_map_mvh_with_grad()的中选取插值候选块的思路进行优化，修改如下：
# 原来的方法，不管(x, y)落于块内部的哪里，其插值计算来源都是固定的那4块，而现在选取的候选块更合理。
# 1.对地磁块block_size * block_size 的区域再细分为 “田” 4小块；    |1 |2 |
# 2.给出待插值点(x, y)，其必定落在这4小块中的某一块；               |3 |4 |
# 3.若落在小块1，则候选块选取左上角的；落在2则选择右上角的... ...
# 4.而此时候选点P00,P01,P10,P11的坐标改为 候选块的中心点
def get_linear_map_mvh_with_grad_2(mag_map, x, y, block_size):
    b_x = int(x // block_size)
    b_y = int(y // block_size)
    num_b_x = len(mag_map)
    num_b_y = 0 if num_b_x == 0 else len(mag_map[0])

    if -1 < b_x < num_b_x and -1 < b_y < num_b_y and mag_map[b_x][b_y][0] != -1 and mag_map[b_x][b_y][1] != -1:
        # 根据x,y所处区域，获取候选块P00,P10,P01,P11的下标
        b_x0 = b_x - 1 if x % block_size <= block_size / 2 else b_x
        b_y0 = b_y - 1 if y % block_size <= block_size / 2 else b_y
        b_x1 = b_x0 + 1
        b_y1 = b_y0 + 1
        # 根据块下标获取块中心坐标：x_center = b_x * bs + bs/2
        half_bs = block_size / 2
        x0 = b_x0 * block_size + half_bs
        x1 = b_x1 * block_size + half_bs
        y0 = b_y0 * block_size + half_bs
        y1 = b_y1 * block_size + half_bs
        # 计算完中心坐标后，再对块下标越界修正，此时取到的地磁值都为非-1
        b_x0 = b_x0 if b_x0 >= 0 else b_x
        b_y0 = b_y0 if b_y0 >= 0 else b_y
        b_x1 = b_x1 if b_x1 < num_b_x else b_x
        b_y1 = b_y1 if b_y1 < num_b_y else b_y
        m_p00 = mag_map[b_x0][b_y0]
        m_p01 = mag_map[b_x0][b_y1]
        m_p10 = mag_map[b_x1][b_y0]
        m_p11 = mag_map[b_x1][b_y1]
        # 带入论文公式4.2、4.3、4.4计算地磁插值、地磁双方向梯度
        m_pxy = (y - y0) * ((x - x0) * m_p11 + (x1 - x) * m_p01) / ((x1 - x0) * (y1 - y0)) + \
                (y1 - y) * ((x - x0) * m_p10 + (x1 - x) * m_p00) / ((x1 - x0) * (y1 - y0))
        grad_px = (y - y0) * (m_p11 - m_p01) / ((x1 - x0) * (y1 - y0)) + (y1 - y) * (m_p10 - m_p00) / (
                (x1 - x0) * (y1 - y0))
        grad_py = (x - x0) * (m_p11 - m_p10) / ((x1 - x0) * (y1 - y0)) + (x1 - x) * (m_p01 - m_p00) / (
                (x1 - x0) * (y1 - y0))

        return m_pxy, np.array([[[grad_px[0], grad_py[0]]], [[grad_px[1], grad_py[1]]]])

    # print("Out of Mag Map!")
    return np.array([-1, -1]), np.array([[[-1, -1]], [[-1, -1]]])


# 坐标变换函数，论文公式4.7。求x1, y1关于transfer的偏导矩阵，论文公式4.13
# 输入：变换向量transfer=[_x, _y, _angle(弧度，绕PDR坐标系的(0, 0)逆时针)]，被变换坐标PDR(x0,y0)
# 输出：转换后的坐标 x1, y1，偏导矩阵 grad_xy(2*3)
# NOTE: t_angle是弧度！不是°。 degrees(x) : 将弧度转化为角度  radians(x) : 将角度转化为弧度。
def transfer_axis_with_grad(transfer, x0, y0):
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
    grad_xy = np.array([[1, 0, -math.sin(_angle) * x0 - math.cos(_angle) * y0],
                        [0, 1, math.cos(_angle) * x0 - math.sin(_angle) * y0]])
    return ans[0][0], ans[1][0], grad_xy


# 输入：轨迹序列，转换向量
# 输出：转换后的轨迹数组
def transfer_axis_of_xy_seq(pdr_xy, transfer):
    map_xy = []
    for xy in pdr_xy:
        x, y, grad = transfer_axis_with_grad(transfer, xy[0], xy[1])
        map_xy.append([x, y])
    return np.array(map_xy)


# 计算残差平方和，论文公式4.8
# 输入：重采样的：PDR实时地磁序列M1[M][mv, mh]，Mag_Map地磁序列M2[M][mv, mh]
# 输出：残差平方和 S
# 实现：欧氏距离
def cal_loss(mag_arr_1, mag_arr_2):
    s = 0
    for m1, m2 in zip(mag_arr_1, mag_arr_2):
        s += (m1[0] - m2[0]) ** 2 + (m1[1] - m2[1]) ** 2
    return s


# 对cal_loss()的优化：
# 将地磁序列减去其均值后再计算残差，认为 相同形状、不同模值 的俩序列的匹配度应该更高(loss更小)。
def cal_mean_loss(mag_arr_1, mag_arr_2):
    s = 0
    mv_arr_1 = remove_mean(mag_arr_1[:, 0])
    mh_arr_1 = remove_mean(mag_arr_1[:, 1])
    mv_arr_2 = remove_mean(mag_arr_2[:, 0])
    mh_arr_2 = remove_mean(mag_arr_2[:, 1])
    num = 0
    for mv_1, mh_1, mv_2, mh_2 in zip(mv_arr_1, mh_arr_1, mv_arr_2, mh_arr_2):
        num += 1
        s += (mv_1 - mv_2) ** 2 + (mh_1 - mh_2) ** 2
    return s / num


# 高斯牛顿迭代法，实现论文公式4.11 4.12
# 实现：仅计算两点之间的高斯牛顿迭代_transfer。 矩阵转置 arr.transpose，逆np.linalg.inv(arr)
# 输入：地磁梯度矩阵mag_map_grads[2][1×2]， 坐标梯度矩阵xy_grad[2×3]，重采样的PDR地磁mag_p[mv, mh]、指纹库的地磁mag_m[mv, mh]
# 输出：单点迭代结果向量_transfer[3×1] --> [1×3]
# NOTE：[1,2,3].shape不是[1×3]而是[0×3]， [[1,2,3]]才是[1×3] ！！！
def cal_GaussNewton_increment(mag_map_grad, xy_grad, mag_P, mag_M, matrix_H_inverse):
    m0 = np.dot(mag_map_grad[0], xy_grad)
    m1 = np.dot(mag_map_grad[1], xy_grad)
    _transfer = np.dot(matrix_H_inverse,
                       np.dot(m0.transpose(), mag_P[0] - mag_M[0]) +
                       np.dot(m1.transpose(), mag_P[1] - mag_M[1]))
    return _transfer.transpose()[0]


# 计算高斯牛顿公式计算过程中的中间矩阵H
# 输入：地磁梯度矩阵mag_map_grads[2][1×2]， 坐标梯度矩阵xy_grad[2×3]
# 输出：计算后的中间矩阵H
def cal_matrix_H(mag_map_grad, xy_grad):
    m0 = np.dot(mag_map_grad[0], xy_grad)
    m1 = np.dot(mag_map_grad[1], xy_grad)
    matrix_H = np.dot(m0.transpose(), m0) + np.dot(m1.transpose(), m1)
    return matrix_H


# 注意全流程的时候进行-1检查！
# 匹配全流程：该函数不会修改sparse_pdr_xy_mvh的内容
# 输入：上一次迭代的transfer(i)向量，缓冲池给的稀疏序列sparse_PDR_Mag[M]，最终的地磁地图Mag_Map[i][j][mv, mh](存在-1)
# 输出：迭代越界失败标志，上一次迭代的transfer(i)对应残差平方和、坐标序列xy(i)， 迭代后的 transfer(i+1)，
def cal_new_transfer_and_last_loss_xy(transfer, sparse_pdr_xy_mvh, mag_map, block_size, step):
    # 1、将sparse_PDR_mag里的PDR x,y 根据 last_transfer 转换坐标并计算坐标梯度，得到 map_xy, xy_grad
    pdr_xy = sparse_pdr_xy_mvh[:, 0:2]
    pdr_mvh = sparse_pdr_xy_mvh[:, 2:4]
    map_xy = []
    xy_grads = []
    for xy in pdr_xy:
        x, y, grad = transfer_axis_with_grad(transfer, xy[0], xy[1])
        map_xy.append([x, y])
        xy_grads.append(grad)
    map_xy = np.array(map_xy)
    xy_grads = np.array(xy_grads)

    # 2、由map_xy到mag_map中获取 map_mvh[M][mv, mh], mag_map_grads[M][2][[1×2]]
    # 若获取到了-1，怎么办？表示到了地图外面，此种迭代方向错误-->提前结束迭代
    # NOTE: 此时map_mvh可能为空
    out_of_map = False
    map_mvh = []
    mag_map_grads = []
    for xy in map_xy:
        mvh, grad = get_linear_map_mvh_with_grad_2(mag_map, xy[0], xy[1], block_size)
        if mvh[0] == -1 or mvh[1] == -1:
            # print("The out point is:", xy, ", [", xy // block_size, "]")
            out_of_map = True
            break
        map_mvh.append(mvh)
        mag_map_grads.append(grad)

    map_mvh = np.array(map_mvh)
    mag_map_grads = np.array(mag_map_grads)

    # 3、计算残差平方和last_loss，如果out_of_map = True，则last_loss无效
    loss = cal_mean_loss(pdr_mvh, map_mvh) if not out_of_map else None

    # 4、由cal_GaussNewton_increment(mag_map_grad, xy_grad, mag_P, mag_M, matrix_H_inverse) * step计算 _transfer
    new_transfer = transfer.copy()
    #    4.1 计算 H 及其 逆
    matrix_H = np.zeros([3, 3], dtype=float)
    for mag_map_grad, xy_grad in zip(mag_map_grads, xy_grads):
        matrix_H += cal_matrix_H(mag_map_grad, xy_grad)

    try:
        matrix_H_inverse = np.linalg.inv(matrix_H)
    except np.linalg.LinAlgError:
        matrix_H_inverse = np.linalg.pinv(matrix_H)
    #    4.2 计算 _transfer 及其总和
    for mag_map_grad, xy_grad, mag_P, mag_M in zip(mag_map_grads, xy_grads, pdr_mvh, map_mvh):
        _transfer = cal_GaussNewton_increment(mag_map_grad, xy_grad, mag_P, mag_M, matrix_H_inverse) * step
        new_transfer[0] += _transfer[0]
        new_transfer[1] += _transfer[1]
        new_transfer[2] += _transfer[2]

    # NOTE:如果out_of_map = True，则last_loss无效
    return out_of_map, loss, map_xy, new_transfer.copy()


# ----------------PDR相关--------------------------------------------------------------------------------------------
# PDR window size = 200，simple frequency=200Hz/s，sliding window size=10
# 而PDR模型输出为该窗口的平均速度v，认为该1秒的窗口速度相同，每 10/200s = 0.05s输出一速度
# 得到的PDR_x,y是该速度v*0.05s，而我们的PDR程序第一个200窗口乘的是0.05s而不是1.0s
# 所以PDR_x,y[0..i..]对应的时间应该为 i*0.05s，对应的数据帧为 i*10
# 一个PDR_x,y坐标应该对应一个地磁值，和根据距离下采样时保持一致，该地磁值也应该用平均值！
# 和simpl_buffer里的一致，达到距离阈值，则返回该段均值和该段mid坐标
# ***所以PDR_x,y[i]对应的地磁 = mean( raw_mag[ i*10-5 : i*10 + 5] ); 不包括 +5


# 输入的data_xy变为不同频率的PDR_xy的缓冲池函数，区别在于内置了对原始数据与PDRxy数据之间的对齐、平均的操作
# 输入：buffer_dis: 缓冲池的距离, down_sip_dis: 下采样距离, data_quat: gameRotationVector, data_mag: 磁力向量集合,
#     + data_xy变为频率不同的PDR_xy，+ PDR轨迹频率pdr_frequency=20, angles/mag采样频率sampling_frequency=200
# 输出：多条匹配序列[?][M][x,y, mv, mh, PDRindex]
# 平均：当频率为200与20时，PDR_x,y[i]对应的地磁 = mean( raw_data[ i*10-5 : i*10 + 5] ); 不包括 +5。
def samples_buffer_PDR(buffer_dis, down_sip_dis,
                       data_quat, data_mag, pdr_xy,
                       do_filter=False,
                       lowpass_filter_level=3,
                       pdr_imu_align_size=10):
    # 0. 检查不合理的pdr和imu的对齐参数
    if pdr_imu_align_size < 1:
        print("Wrong pdr_imu_align_size in samples_buffer_PDR")
        return None

    # 1.计算mv,mh分量：得到[N][mv, mh]
    arr_mv_mh = get_2d_mag_qiu(data_quat, data_mag)

    # 2.滤波：对匹配序列中的地磁进行滤波，在下采样之前滤波，下采样之后太短了不能滤波
    if do_filter:
        mv_filtered_emd = lowpass_emd(arr_mv_mh[:, 0], lowpass_filter_level)
        mh_filtered_emd = lowpass_emd(arr_mv_mh[:, 1], lowpass_filter_level)
        arr_mv_mh = np.vstack((mv_filtered_emd, mh_filtered_emd)).transpose()

    # 3.对齐：将imu高频arr_mv_mh 变为和 低频pdr_xy 对齐、平均后的和PDR_xy 的arr_mv_mh
    mv_mh_pdr_list = []
    for pdr_i in range(0, len(pdr_xy)):
        raw_i = pdr_i * pdr_imu_align_size
        raw_i_start = raw_i - int(pdr_imu_align_size / 2)
        raw_i_end = raw_i + int(pdr_imu_align_size / 2)
        if raw_i_start < 0:
            raw_i_start = 0
        if raw_i_end > len(data_mag):
            raw_i_end = len(data_mag)
        if raw_i_start < raw_i_end:
            mv_mh_pdr_list.append([np.mean(arr_mv_mh[raw_i_start:raw_i_end, 0]),
                                   np.mean(arr_mv_mh[raw_i_start:raw_i_end, 1])])
        else:
            break
    mv_mh_pdr_arr = np.array(mv_mh_pdr_list)

    # 4. for遍历PDR_xy，计算距离，达到down_sip_dis/2记录 i_mid，达到 down_sip_dis记录 i_end并计算down_sampling
    i_start = 0
    i_mid = i_start
    dis_sum_temp = 0
    dis_sum_all = 0
    dis_mid = down_sip_dis / 2
    match_seq_list = []
    match_seq = []
    # 使用len(arr_mv_mh)，而不是 len(PDR_xy)，因为经过 3. 后，前者有可能小于后者
    for i in range(1, len(mv_mh_pdr_arr)):
        dis_increment = math.hypot(pdr_xy[i][0] - pdr_xy[i - 1][0], pdr_xy[i][1] - pdr_xy[i - 1][1])
        dis_sum_temp += dis_increment
        dis_sum_all += dis_increment
        if dis_sum_temp >= down_sip_dis:
            match_seq.append(down_sampling(i_start, i_mid, i, pdr_xy, mv_mh_pdr_arr))
            i_start = i
            i_mid = i_start
            dis_sum_temp = 0
            # dis_sum_temp -= down_sip_dis 这样写虽然很真实，但会导致dis_sum_temp与i_mid_xy含义不一致，
            # 之前的会影响后续的，而且在dis_sum_all+= dis_sum_temp的时候会重复加上之前未清0的距离
        else:
            if dis_sum_temp >= dis_mid:
                i_mid = i

        if dis_sum_all >= buffer_dis:
            # dis_sum_all -= buffer_dis 这样写，之前的会影响后续的
            dis_sum_all = 0
            match_seq_list.append(np.array(match_seq))
            match_seq.clear()

    match_seq_list.append(match_seq)  # Don't change to numpy array, because the len(match_seq) is different
    return match_seq_list


# TEST 测试slide_dis == buffer_dis时（退化为无滑动窗口），本方法与方法[samples_buffer_PDR]结果不同的原因：
#     由于本方法下采样与填充buffer的逻辑分离，在计算缓冲池距离增量时，本方法的距离增加粒度是下采样后的粒度。
#     而方法[samples_buffer_PDR]中，下采样与填充buffer的逻辑同时进行，buffer距离增加粒度是下采样前的粒度。
#     所以方法[samples_buffer_PDR]的buffer距离增量可能会比本方法先一步达到buffer_dis，导致len(match_seq)不一致（本方法可能多1）
def samples_buffer_with_pdr_and_slidewindow(buffer_dis, downsampling_dis,
                                            data_quat, data_mag, pdr_xy,
                                            do_filter=False,
                                            lowpass_filter_level=3,
                                            pdr_imu_align_size=10,
                                            slide_step=2,
                                            slide_block_size=0.25):
    # 0. 检查不合理参数（非完全检查）
    if pdr_imu_align_size < 1:
        print("*Error: Wrong pdr_imu_align_size [samples_buffer_with_pdr_and_slidewindow]")
        return None

    slide_dis = slide_step * slide_block_size  # 每次滑窗的距离
    if slide_dis > buffer_dis:  # 窗口滑动的距离大于窗口本身
        print("*Error: slide distance {0} is longer than buffer distance {1} [samples_buffer_with_pdr_and_slidewindow]"
              .format(slide_dis, buffer_dis))
        return None

    # 1.计算mv,mh分量：得到[N][mv, mh]
    arr_mv_mh_mm = get_2d_mag_qiu(data_quat, data_mag)

    # 2.滤波：对匹配序列中的地磁进行滤波，在下采样之前滤波，下采样之后太短了不能滤波
    if do_filter:
        mv_filtered_emd = lowpass_emd(arr_mv_mh_mm[:, 0], lowpass_filter_level)
        mh_filtered_emd = lowpass_emd(arr_mv_mh_mm[:, 1], lowpass_filter_level)
        arr_mv_mh_mm = np.vstack((mv_filtered_emd, mh_filtered_emd)).transpose()

    # 3.对齐：将imu高频arr_mv_mh 变为和 低频pdr_xy 对齐、平均后的arr_mv_mh
    arr_mv_mh_aligned_to_pdr = align_mv_mh_to_pdr(arr_mv_mh_mm, pdr_xy, pdr_imu_align_size)

    # 4. 下采样，获取下采样后的 位置-磁场 序列[N3][pdr_x, pdr_y, mmv, mmh, pdr_xy_index]
    xy_mvh_downsampled_list = []
    dis_sum_temp = 0
    i_start = 0
    i_mid = i_start
    mid_downsampling_dis = downsampling_dis / 2
    # 使用len(arr_mv_mh_aligned_to_pdr)，而不是len(PDR_xy)，因为经过3.后:前者有可能<后者
    for i in range(1, len(arr_mv_mh_aligned_to_pdr)):
        # 当累计距离达到downsampling_dis就执行下采样，添加到pdr_xy_mag_vh_list
        dis_sum_temp += math.hypot(pdr_xy[i][0] - pdr_xy[i - 1][0], pdr_xy[i][1] - pdr_xy[i - 1][1])
        if dis_sum_temp >= downsampling_dis:
            xy_mvh_downsampled_list.append(down_sampling(i_start, i_mid, i, pdr_xy, arr_mv_mh_aligned_to_pdr))
            dis_sum_temp = 0
            i_start = i
            i_mid = i_start
        else:
            if dis_sum_temp >= mid_downsampling_dis:
                i_mid = i

    # 5. 对 xy_mvh_downsampled_list 进行滑动窗口产生匹配段match_seq_list
    window_buffer = []  # 保存一个距离窗口的[pdr_x, pdr_y, mmv, mmh, pdr_xy_index]
    slide_number_queue = queue.Queue()  # 提前保存每个滑窗对应的数据个数
    slide_dis_queue = queue.Queue()  # 保存滑动的距离
    dis_sum_buffer = 0  # 保存当前窗口累计距离，达到buffer_dis清0
    dis_sum_silde = 0  # 保存当前滑动累计距离，达到slide_dis清0
    match_seq_list = []  # 返回给外层的、保存所有待匹配的窗口
    slide_number_list = []  # 返回给外层的、和match_seq_list中的每个match_seq对应的

    # 先填满一个窗口，装入match_seq_list
    index = 0
    last_slide_index = -1  # 上一次记录滑窗的下标，注意初值-1
    window_buffer.append(xy_mvh_downsampled_list[index])
    index += 1
    while dis_sum_buffer < buffer_dis:
        if index >= len(xy_mvh_downsampled_list):  # 如果整个轨迹都达不到一个窗口
            print("*Error: the whole distance is too short [samples_buffer_with_pdr_and_slidewindow]")
            return None

        dis_increment = math.hypot(xy_mvh_downsampled_list[index][0] - xy_mvh_downsampled_list[index - 1][0],
                                   xy_mvh_downsampled_list[index][1] - xy_mvh_downsampled_list[index - 1][1])
        dis_sum_silde += dis_increment
        dis_sum_buffer += dis_increment
        window_buffer.append(xy_mvh_downsampled_list[index])
        # 此if要放在dis增加的后面，否则当最后一次循环时dis_sum_silde达到slide_dis时不会进入该判断
        if dis_sum_silde >= slide_dis:
            slide_number_queue.put(index - last_slide_index)
            slide_dis_queue.put(dis_sum_silde)
            dis_sum_silde = 0
            last_slide_index = index
        index += 1  # 注意该操作不能放在if前

    match_seq_list.append(np.array(window_buffer))
    slide_number = slide_number_queue.get()
    slide_number_list.append(slide_number)
    del window_buffer[0: slide_number]  # 删除窗口头部滑动数据，代表滑动窗口
    dis_sum_buffer -= slide_dis_queue.get()  # 窗口滑动（舍弃）的距离

    # 继续滑动窗口
    for index_2 in range(index, len(xy_mvh_downsampled_list)):
        dis_increment = math.hypot(xy_mvh_downsampled_list[index_2][0] - xy_mvh_downsampled_list[index_2 - 1][0],
                                   xy_mvh_downsampled_list[index_2][1] - xy_mvh_downsampled_list[index_2 - 1][1])
        dis_sum_silde += dis_increment
        dis_sum_buffer += dis_increment
        window_buffer.append(xy_mvh_downsampled_list[index_2])

        if dis_sum_silde >= slide_dis:
            slide_number_queue.put(index_2 - last_slide_index)
            slide_dis_queue.put(dis_sum_silde)
            dis_sum_silde = 0
            last_slide_index = index_2

        if dis_sum_buffer >= buffer_dis:
            match_seq_list.append(np.array(window_buffer))
            slide_number = slide_number_queue.get()
            slide_number_list.append(slide_number)
            del window_buffer[0: slide_number]
            dis_sum_buffer -= slide_dis_queue.get()

    return match_seq_list, slide_number_list


# 将高频的磁场指纹平均对齐到pdr坐标
# 输入：通过get_2d_mag_qiu()返回的磁场分量arr_mv_mh_mm，低频pdr轨迹pdr_xy，pdr滑动窗口大小pdr_imu_align_size
# 返回：[len(pdr_xy)][mv, mh]
def align_mv_mh_to_pdr(arr_mv_mh_mm, pdr_xy, pdr_imu_align_size):
    mv_mh_aligned_to_pdr = []
    for pdr_i in range(0, len(pdr_xy)):
        raw_i = (pdr_i + 1) * pdr_imu_align_size
        raw_i_start = raw_i - int(pdr_imu_align_size / 2)
        raw_i_end = raw_i + int(pdr_imu_align_size / 2)
        if raw_i_start < 0:
            raw_i_start = 0
        if raw_i_end > len(arr_mv_mh_mm):
            raw_i_end = len(arr_mv_mh_mm)
        if raw_i_start < raw_i_end:
            # 这里只用了mv和mh
            mv_mh_aligned_to_pdr.append([np.mean(arr_mv_mh_mm[raw_i_start:raw_i_end, 0]),
                                         np.mean(arr_mv_mh_mm[raw_i_start:raw_i_end, 1])])
        else:
            break
    arr_mv_mh_aligned_to_pdr = np.array(mv_mh_aligned_to_pdr)
    return arr_mv_mh_aligned_to_pdr


# 候选transfer向量生成器：
# 输入一个向量，输出以该向量为中心，产生的一组用以小区域遍历的候选项。
# 输出的候选向量应该按由近到远的辐射顺序
# 可自定义参数: transfer各维度分别的增减粒度、增减次数
# 输入：初始向量original_transfer[△x, △y(米), △angle(弧度)]，
#       自定义参数config[2][3]=[[x,y(米),angle(弧度)增减粒度],[x,y,angle增减次数]]
# 输出：满足要求的候选transfer_candidates[m][3] （包括original_transfer）
def produce_transfer_candidates_ascending(original_transfer, config):
    transfer_candidates = []
    x_candidates = []
    y_candidates = []
    angle_candidates = []
    # for循环增减次数次，向各个参数list中添加
    x_candidates.append(original_transfer[0])
    for tx in range(1, config[1][0]):
        x_candidates.append(original_transfer[0] + tx * config[0][0])
        x_candidates.append(original_transfer[0] - tx * config[0][0])

    y_candidates.append(original_transfer[1])
    for ty in range(1, config[1][1]):
        y_candidates.append(original_transfer[1] + ty * config[0][1])
        y_candidates.append(original_transfer[1] - ty * config[0][1])

    angle_candidates.append(original_transfer[2])
    for ta in range(1, config[1][2]):
        angle_candidates.append(original_transfer[2] + ta * config[0][2])
        angle_candidates.append(original_transfer[2] - ta * config[0][2])

    # 将这些候选分量按变换距离升序重新排列组和。x,y,z_candidates内部已按升序
    # 所以，以(i_x + i_y + i_angle)作为新组合的下标，
    temp_list = []
    len_x = len(x_candidates)
    len_y = len(y_candidates)
    len_angle = len(angle_candidates)

    for i_t in range(0, len_x + len_y + len_angle):
        temp_list.append([])

    for i_x in range(0, len_x):
        for i_y in range(0, len_y):
            for i_angle in range(0, len_angle):
                temp_list[i_x + i_y + i_angle].append([x_candidates[i_x], y_candidates[i_y], angle_candidates[i_angle]])

    # 将temp_list中的范围升序结果提取到transfer_candidates中。NOTE：len(temp_list[i])不全相等！
    for temp in temp_list:
        for t in temp:
            transfer_candidates.append(t)

    return np.array(transfer_candidates)


class SearchPattern(Enum):
    FULL_DEEP = 0
    BREAKE_ADVANCED_AND_USE_LAST_WHEN_FAILED = 1
    BREAKE_ADVANCED_AND_USE_SECOND_LOSS_WHEN_FAILED = 2


# 输入：用于生成transfer_candidates：初始变换向量original_transfer，范围参数area_config，
#     用于高斯牛顿迭代的：待匹配序列match_seq，地磁地图mag_map，块大小block_size，迭代步长step，最大迭代次数max_iteration
#     用于筛选候选项的：目标损失target_loss
#      upper_limit_of_gaussnewteon 当前参数下的高斯牛顿迭代可降低的loss上限，是一个经验值，避免注定会搜索失败的候选项，提高性能！
#      search_pattern 是否在找到小于target loss的transfer就马上退出
# 输出：最终选择的Transfer（当小范围寻找失败时，则返回original_transfer），以及其对应的map_xy
def produce_transfer_candidates_and_search(start_transfer, area_config,
                                           match_seq,
                                           mag_map, block_size,
                                           step, max_iteration, target_loss, upper_limit_of_gaussnewteon,
                                           search_pattern=SearchPattern.BREAKE_ADVANCED_AND_USE_LAST_WHEN_FAILED):
    max_iteration += 1

    # 1.生成小范围的所有transfer_candidates（包括start_transfer，且范围由近到远）
    transfer_candidates = produce_transfer_candidates_ascending(start_transfer, area_config)

    # 2.遍历transfer_candidates进行高斯牛顿，结果添加到候选集candidates_loss_xy_tf
    candidates_loss_xy_tf = []

    for transfer in transfer_candidates:
        # 根据经验判断当前loss是否已经超出了高斯牛顿迭代的优化能力
        out_of_map, start_loss, not_use_map_xy, not_use_transfer = cal_new_transfer_and_last_loss_xy(
            transfer, match_seq, mag_map, block_size, step)
        if out_of_map or start_loss - target_loss > upper_limit_of_gaussnewteon:
            # 超过了高斯牛顿的迭代能力，不用继续迭代了，直接下一个candidate
            continue

        last_loss_xy_tf_num = None
        loss_list = []
        # 高斯牛顿迭代
        TEST_start_transfer = transfer.copy()
        for iter_num in range(1, max_iteration):
            out_of_map, loss, map_xy, next_transfer = cal_new_transfer_and_last_loss_xy(
                transfer, match_seq, mag_map, block_size, step)
            # 界内
            if not out_of_map:
                loss_list.append(loss)
                if search_pattern == SearchPattern.BREAKE_ADVANCED_AND_USE_SECOND_LOSS_WHEN_FAILED:
                    last_loss_xy_tf_num = [loss, map_xy, transfer, iter_num]
                # 损失达标
                if loss <= target_loss:
                    last_loss_xy_tf_num = [loss, map_xy, transfer, iter_num]
                    if search_pattern == SearchPattern.BREAKE_ADVANCED_AND_USE_LAST_WHEN_FAILED \
                            or search_pattern == SearchPattern.BREAKE_ADVANCED_AND_USE_SECOND_LOSS_WHEN_FAILED:
                        print("\t\t.search Succeed and break in advanced. Final loss = ", loss)

                        # TEST
                        # print("Start transfer = ",
                        #                         #       [TEST_start_transfer[0], TEST_start_transfer[1], math.degrees(TEST_start_transfer[2])],
                        #                         #       "Last transfer = ", [transfer[0], transfer[1], math.degrees(transfer[2])],
                        #                         #       "Change = ", [(transfer[0] - TEST_start_transfer[0]),
                        #                         #                     (transfer[1] - TEST_start_transfer[1]),
                        #                         #                     math.degrees(transfer[2] - TEST_start_transfer[2])],
                        #                         #       "Iter num = ", iter_num,
                        #                         #       "Mean change = ", [(transfer[0] - TEST_start_transfer[0]) / iter_num,
                        #                         #                          (transfer[1] - TEST_start_transfer[1]) / iter_num,
                        #                         #                          math.degrees(transfer[2] - TEST_start_transfer[2]) / iter_num],
                        #                         #       "Start loss = ", start_loss,
                        #                         #       "Final loss = ", loss)

                        return transfer, map_xy
                else:  # loss > target and not out of map, continue try next transfer.
                    transfer = next_transfer
            else:
                break

        # TEST
        # print("Start transfer = ",
        #       [TEST_start_transfer[0], TEST_start_transfer[1], math.degrees(TEST_start_transfer[2])],
        #       "Last transfer = ", [transfer[0], transfer[1], math.degrees(transfer[2])],
        #       "Change = ", [(transfer[0] - TEST_start_transfer[0]),
        #                     (transfer[1] - TEST_start_transfer[1]),
        #                     math.degrees(transfer[2] - TEST_start_transfer[2])],
        #       "Iter num = ", iter_num,
        #       "Mean change = ", [(transfer[0] - TEST_start_transfer[0]) / iter_num,
        #                          (transfer[1] - TEST_start_transfer[1]) / iter_num,
        #                          math.degrees(transfer[2] - TEST_start_transfer[2]) / iter_num],
        #       "Start loss = ", start_loss,
        #       "Final loss = ", loss)

        if last_loss_xy_tf_num is not None:
            candidates_loss_xy_tf.append(last_loss_xy_tf_num)
    # 如果选择了提前结束，但是到了这一步，表示寻找失败
    if search_pattern == SearchPattern.BREAKE_ADVANCED_AND_USE_LAST_WHEN_FAILED:
        print("\t\t.Failed search, use last transfer. Final loss = ", loss)
        return start_transfer, transfer_axis_of_xy_seq(match_seq, start_transfer)

    # if search_pattern == SearchPattern.FULL_DEEP or BREAKE_ADVANCED_AND_USE_SECOND_LOSS_WHEN_FAILED:
    # 选出候选集中Loss最小的项，返回其transfer；
    # 若无候选项，则表示小范围寻找失败，返回original_transfer
    transfer = None
    min_loss = None
    min_xy = None
    for c in candidates_loss_xy_tf:
        if min_loss is None or c[0] < min_loss:
            min_loss = c[0]
            min_xy = c[1]
            transfer = c[2]

    if transfer is None or (
            search_pattern == SearchPattern.BREAKE_ADVANCED_AND_USE_SECOND_LOSS_WHEN_FAILED and min_loss > target_loss * 2):
        print("\t\t.Failed search, use last transfer.Final loss = ", min_loss)
        return start_transfer, transfer_axis_of_xy_seq(match_seq, start_transfer)
    else:
        print("\t\t.Found min, final loss = ", min_loss)
        return transfer, min_xy


# 初始固定区域搜索，论文原文如下：
# “进入磁场特征匹配算法阶段，考虑到高斯牛顿迭代搜索是局部最优方法，
# 第一次匹配对固定区域进行遍历搜索，避免初始条件不够精确造成匹配结果非全局最优，
# 其后的匹配则可直接利用 PDR 推算结果作为匹配搜索初值；”
# 若不固定区域约束，全图遍历时间复杂度可达到百万级别......
# 给定起始约束，这里的起始约束为：机房的N个入口（门）坐标（已知全图消息，且门必然是在图边界处）.
# 此时轨迹只会在该门的一边 且 一段必须连着坐标。所以{△x, △y, △angle(弧度)}只需考虑N * angle 180°变换内的约束。
# 大大减少了匹配时间复杂度、提高了准确率.
# 输入：entrance_list: [0, map_size_x][0,map_size_y]坐标系下的入口坐标
#      match_seq:待匹配的序列[N][x,y, mv, mh]
def inital_full_deep_search(entrances, match_seq,
                            mag_map, block_size,
                            step, max_iteration, target_loss,
                            reduce_dis=1):
    max_iteration += 1
    # 从多个入口中找出loss最小的transfer
    match_seq_backup = match_seq.copy()
    min_loss = None
    min_transfer = None

    # 构建0-360°，1.°粒度的transfer
    transfer_candidates = []
    for angle in np.arange(0, 360, 2):
        transfer_candidates.append([0, 0, math.radians(angle)])

    start_x = match_seq[0][0]
    start_y = match_seq[0][1]

    while True:
        for entrance in entrances:
            for transfer_candidate in transfer_candidates:
                # 先将xy轨迹变换到entrance坐标，不修改原来的match_seq
                # 构建transfer，先用transfer_candidate旋转start_x,y，然后将旋转后的点平移到entrance，该平移量+旋转角度=transfer
                new_xy = transfer_axis_of_xy_seq([[start_x, start_y]], transfer_candidate)
                transfer = np.array([entrance[0] - new_xy[0][0], entrance[1] - new_xy[0][1], transfer_candidate[2]])

                # 将该start_transfer应用到match_seq_copy，进行高斯牛顿迭代，找到最小loss
                out_of_map, start_loss, not_use_map_xy, not_use_transfer = cal_new_transfer_and_last_loss_xy(
                    transfer, match_seq, mag_map, block_size, step)

                if out_of_map:
                    # 出界
                    continue

                for iter_num in range(1, max_iteration):
                    out_of_map, loss, map_xy, next_transfer = cal_new_transfer_and_last_loss_xy(
                        transfer, match_seq, mag_map, block_size, step)  # 该函数不会修改match_seq
                    if not out_of_map:
                        transfer = next_transfer  # 继续迭代
                        if min_loss is None or loss < min_loss:
                            min_loss = loss
                            min_transfer = transfer.copy()
                    else:
                        break

        if min_transfer is not None and min_loss < target_loss:
            # 非空且非空的结果在目标loss之内，则认为初始化搜索成功
            break
        else:
            # 将match_seq减掉末尾的1m后继续匹配，如果减到0米仍不行，则认为存在问题，返回全None
            abondon_num = cal_dis_num_from_tail(match_seq, reduce_dis)
            # print(abondon_num, ',', len(match_seq))
            if abondon_num >= len(match_seq) or abondon_num == 0:
                return None, None, None
            match_seq = match_seq[0:len(match_seq) - abondon_num, :]
            print('初始匹配段减1米')
            # del match_seq[len(match_seq) - abondon_num: len(match_seq)]

    return min_transfer, transfer_axis_of_xy_seq(match_seq_backup, min_transfer), min_loss

def inital_full_deep_search_with_angleRange(entrances, angleRange, match_seq,
                            mag_map, block_size,
                            step, max_iteration, target_loss,
                            reduce_dis=1):
    max_iteration += 1
    # 从多个入口中找出loss最小的transfer
    match_seq_backup = match_seq.copy()
    min_loss = None
    min_transfer = None

    # 构建0-360°，1.°粒度的transfer
    transfer_candidates = []
    for angle in np.arange(0, 360, 2):
        transfer_candidates.append([0, 0, math.radians(angle)])

    start_x = match_seq[0][0]
    start_y = match_seq[0][1]

    while True:
        for entrance in entrances:
            for transfer_candidate in transfer_candidates:
                # 先将xy轨迹变换到entrance坐标，不修改原来的match_seq
                # 构建transfer，先用transfer_candidate旋转start_x,y，然后将旋转后的点平移到entrance，该平移量+旋转角度=transfer
                new_xy = transfer_axis_of_xy_seq([[start_x, start_y]], transfer_candidate)
                transfer = np.array([entrance[0] - new_xy[0][0], entrance[1] - new_xy[0][1], transfer_candidate[2]])

                # 将该start_transfer应用到match_seq_copy，进行高斯牛顿迭代，找到最小loss
                out_of_map, start_loss, not_use_map_xy, not_use_transfer = cal_new_transfer_and_last_loss_xy(
                    transfer, match_seq, mag_map, block_size, step)

                if out_of_map:
                    # 出界
                    continue

                for iter_num in range(1, max_iteration):
                    out_of_map, loss, map_xy, next_transfer = cal_new_transfer_and_last_loss_xy(
                        transfer, match_seq, mag_map, block_size, step)  # 该函数不会修改match_seq
                    if not out_of_map:
                        transfer = next_transfer  # 继续迭代
                        if min_loss is None or loss < min_loss:
                            min_loss = loss
                            min_transfer = transfer.copy()
                    else:
                        break

        if min_transfer is not None and min_loss < target_loss:
            # 非空且非空的结果在目标loss之内，则认为初始化搜索成功
            break
        else:
            # 将match_seq减掉末尾的1m后继续匹配，如果减到0米仍不行，则认为存在问题，返回全None
            abondon_num = cal_dis_num_from_tail(match_seq, reduce_dis)
            # print(abondon_num, ',', len(match_seq))
            if abondon_num >= len(match_seq) or abondon_num == 0:
                return None, None, None
            match_seq = match_seq[0:len(match_seq) - abondon_num, :]
            print('初始匹配段减1米')
            # del match_seq[len(match_seq) - abondon_num: len(match_seq)]

    return min_transfer, transfer_axis_of_xy_seq(match_seq_backup, min_transfer), min_loss


# 预处理pdr发送过来的数据.将pdr_data_buffer：
# [N][time, [pdr_x, y], [10*[mag x, y, z, quat x, y, z, w]], pdr_index]变为[N][x,y, mv, mh] [N][pdr_index]
# 该函数不会修改pdr_data_buffer指向的内容
# 下采样
def change_pdr_thread_data_to_match_seq(pdr_data_buffer, down_sip_dis, lowpass_level):
    pdr_num = len(pdr_data_buffer)
    if pdr_num == 0:
        return np.empty(shape=[0, 0]), np.empty(shape=[0, 0])

    # 先将pdr_data_buffer中每行中的10个[mag,ori]合并成连续的一段，
    mag_ori_per_num = len(pdr_data_buffer[0][2])
    all_mag_ori_arr = np.empty(shape=[pdr_num * mag_ori_per_num, 7], dtype=float)
    for pi in range(0, pdr_num):
        start_i = pi * mag_ori_per_num
        for pj in range(0, mag_ori_per_num):
            cur_i = start_i + pj
            for pk in range(0, 7):
                all_mag_ori_arr[cur_i][pk] = pdr_data_buffer[pi][2][pj][pk]

    # 对这连续的一段求mv\mh后滤波
    mvh_arr = get_2d_mag_qiu(all_mag_ori_arr[:, 3:7], all_mag_ori_arr[:, 0:3])
    mv_filtered_emd = lowpass_emd(mvh_arr[:, 0], lowpass_level)
    mh_filtered_emd = lowpass_emd(mvh_arr[:, 1], lowpass_level)
    mvh_arr = np.vstack((mv_filtered_emd, mh_filtered_emd)).transpose()

    # 滤波后对mv/mh每10个平均后还给对应pdr_xy，得到match_seq_arr[N][x,y, mv,mh, pdr_index]
    match_seq_arr = np.empty(shape=[pdr_num, 6], dtype=float)
    for pi in range(0, pdr_num):
        match_seq_arr[pi][0] = pdr_data_buffer[pi][1][0]  # x
        match_seq_arr[pi][1] = pdr_data_buffer[pi][1][1]  # y

        start_i, end_i = pi * mag_ori_per_num, (pi + 1) * mag_ori_per_num
        match_seq_arr[pi][2] = np.mean(mvh_arr[start_i:end_i, 0])  # mean(mv)
        match_seq_arr[pi][3] = np.mean(mvh_arr[start_i:end_i, 1])  # mean(mh)

        match_seq_arr[pi][4] = pdr_data_buffer[pi][3]  # pdr_index

        match_seq_arr[pi][5] = pdr_data_buffer[pi][0]  # time

    # 下采样，过程中记录i_mid对应的pdr原始下标
    match_seq_list = []
    pdr_index_list = []
    time_list = []
    dis_sum_temp = 0
    i_start = 0
    i_mid = i_start
    mid_down_sip_dis = down_sip_dis / 2

    for i in range(1, len(match_seq_arr)):
        dis_sum_temp += math.hypot(match_seq_arr[i][0] - match_seq_arr[i - 1][0],
                                   match_seq_arr[i][1] - match_seq_arr[i - 1][1])

        if dis_sum_temp >= down_sip_dis or i == len(match_seq_arr) - 1:
            match_seq_list.append([match_seq_arr[i_mid][0], match_seq_arr[i_mid][1],
                                   np.mean(match_seq_arr[i_start:i, 2]),
                                   np.mean(match_seq_arr[i_start:i, 3])])
            pdr_index_list.append(match_seq_arr[i_mid][4])
            time_list.append(match_seq_arr[i_mid][5])
            dis_sum_temp = 0
            i_start = i
            i_mid = i_start
        else:
            if dis_sum_temp >= mid_down_sip_dis:
                i_mid = i

    return np.array(match_seq_list), np.array(pdr_index_list), np.array(time_list)


# 均值移除
def remove_mean(magSerial):
    return magSerial - np.mean(magSerial)


# 计算某个距离的坐标点的数量，如果目标距离小于等于0，则返回0，如果距离大于最大，则返回len
# xy = [N][x, y]
# 返回的结果可直接用于del [0 : ans]
def cal_dis_num_from_head(xy, target_dis):
    if target_dis <= 0 or len(xy) < 2:
        return 0

    dis = 0
    found = False
    for i in range(1, len(xy)):
        dis += math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
        if dis >= target_dis:
            found = True
            break

    return i if found else len(xy)


# 返回的结果可直接用于 del [len - ans : len]
def cal_dis_num_from_tail(xy, target_dis):
    if target_dis <= 0 or len(xy) < 2:
        return 0

    dis = 0
    found = False
    for i in range(len(xy) - 1, 0, -1):
        dis += math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
        if dis >= target_dis:
            found = True
            break

    return len(xy) - i if found else len(xy)


# -----------计算磁场特征的函数--------------------------------------------------------------------------------------
# 地磁序列标准差D：该值越高，表示特征越丰富
# 输入：磁场序列mag_arr[N][mv, mh] （地磁指纹库磁场序列 or 实测磁场序列）
# 输出：标准差std_deviation_mv, std_deviation_mh
# 算法：1.先计算各分量均值；2.标准差= math.sqrt(1/N * sum(mag_i - mag_mean)**2)
def cal_std_deviation_mag_vh(mag_vh_arr):
    mag_vh_arr = np.array(mag_vh_arr)

    mv_mean = np.mean(mag_vh_arr[:, 0])
    mh_mean = np.mean(mag_vh_arr[:, 1])
    n = len(mag_vh_arr)
    sum_mv_dis = 0
    sum_mh_dis = 0
    for mv_mh in mag_vh_arr:
        sum_mv_dis += (mv_mh[0] - mv_mean) ** 2
        sum_mh_dis += (mv_mh[1] - mh_mean) ** 2

    std_deviation_mv = math.sqrt(1 / n * sum_mv_dis)
    std_deviation_mh = math.sqrt(1 / n * sum_mh_dis)
    std_deviation_all = math.sqrt(std_deviation_mv ** 2 + std_deviation_mh ** 2)
    return std_deviation_mv, std_deviation_mh, std_deviation_all


# 地磁序列相邻点相关系数（总和）：相关系数越大，相邻点越接近，特征越低，所以返回其倒数表示不相关程度。
# 输入：磁场序列mag_arr[N][mv, mh] （地磁指纹库磁场序列 or 实测磁场序列）
# 输出：不相关程度 unsameness = 1/相关系数
# 算法：1.分别计算两个分量的所需值；2.相关系数R = ...
def cal_unsameness_mag_vh(mag_vh_arr):
    mag_vh_arr = np.array(mag_vh_arr)

    std_deviation_mv, std_deviation_mh, not_used_ret = cal_std_deviation_mag_vh(mag_vh_arr)
    n = len(mag_vh_arr)
    mv_mean = np.mean(mag_vh_arr[:, 0])
    mh_mean = np.mean(mag_vh_arr[:, 1])
    k_mv = 1 / ((n - 1) * (std_deviation_mv ** 2))
    k_mh = 1 / ((n - 1) * (std_deviation_mh ** 2))
    sum_temp_mv = 0
    sum_temp_mh = 0

    for i in range(1, n):
        sum_temp_mv += (mag_vh_arr[i][0] - mv_mean) * (mag_vh_arr[i - 1][0] - mv_mean)
        sum_temp_mh += (mag_vh_arr[i][1] - mh_mean) * (mag_vh_arr[i - 1][1] - mh_mean)

    sameness_mv = k_mv * sum_temp_mv
    sameness_mh = k_mh * sum_temp_mh
    unsameness_mv = 1 / sameness_mv
    unsameness_mh = 1 / sameness_mh
    unsameness_all = math.sqrt(unsameness_mv ** 2 + unsameness_mh ** 2)
    return unsameness_mv, unsameness_mh, unsameness_all


# 地磁梯度水平：和标准差类似，越高则特征越丰富
# 输入: mag_map_grads[N][2][1][2] = [N][[[grad_mv_x, grad_mv_y]], [[grad_mh_x, grad_mh_y]]]
# 输出：4分量梯度水平
# NOTE：1.该函数的输入仅使用地磁指纹库中来源的双分量、双方向梯度，无法计算实测低维地磁序列双方向梯度。
#       与loss结合：loss小代表实测序列与指纹库序列相似，所以此时库梯度也能一定程度代表实测梯度水平。
#      2.如何获取该数组中的值：grad_mv_x = mag_map_grads[i][0][0][0]
# 算法：对单个分量，grad_level = math.sqrt(1/N * sum(grad_i**2))
def cal_grads_level_mag_vh(mag_map_grads):
    mag_map_grads = np.array(mag_map_grads)

    n = len(mag_map_grads)
    sum_grad_mv_x = 0
    sum_grad_mv_y = 0
    sum_grad_mh_x = 0
    sum_grad_mh_y = 0

    for grad_all in mag_map_grads:
        sum_grad_mv_x += grad_all[0][0][0] ** 2
        sum_grad_mv_y += grad_all[0][0][1] ** 2
        sum_grad_mh_x += grad_all[1][0][0] ** 2
        sum_grad_mh_y += grad_all[1][0][1] ** 2

    grad_level_mv_x = math.sqrt(1 / n * sum_grad_mv_x)
    grad_level_mv_y = math.sqrt(1 / n * sum_grad_mv_y)
    grad_level_mh_x = math.sqrt(1 / n * sum_grad_mh_x)
    grad_level_mh_y = math.sqrt(1 / n * sum_grad_mh_y)

    grad_level_mv = math.sqrt(grad_level_mv_x ** 2 + grad_level_mv_y ** 2)
    grad_level_mh = math.sqrt(grad_level_mh_x ** 2 + grad_level_mh_y ** 2)
    grad_level_all = math.sqrt(grad_level_mv ** 2 + grad_level_mh ** 2)

    return grad_level_mv, grad_level_mh, grad_level_all


def rebuild_map_from_mvh_files(mag_map_file_path):
    if len(mag_map_file_path) != 2:
        print("Need two files in rebuild_map_from_mvh_files()")
        return None
    mag_map_mv = np.array(np.loadtxt(mag_map_file_path[0], delimiter=','))
    mag_map_mh = np.array(np.loadtxt(mag_map_file_path[1], delimiter=','))
    mag_map = []
    for i in range(0, len(mag_map_mv)):
        temp = []
        for j in range(0, len(mag_map_mv[0])):
            temp.append([mag_map_mv[i][j], mag_map_mh[i][j]])
        mag_map.append(temp)
    mag_map = np.array(mag_map)

    return mag_map

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import math


# 根据块绘制栅格地磁强度图（热力图）
# cmap:YlOrRd
# 输入：[x][y][mv, mh]
def paint_heat_map(arr_mv_mh, num=0, show_mv=True, show_mh=True, save_dir=None):
    if show_mv:
        # plt.figure(figsize=(10, 36))
        plt.figure(figsize=(20, 10))
        plt.title('mag_vertical_' + str(num))
        # sns.set(font_scale=0.8)
        # cmap='YlOrRd'
        sns.heatmap(
            data=arr_mv_mh[:, ::-1, 0].T,
            cmap='jet',
            # annot=True, fmt='.0f',
            mask=arr_mv_mh[:, ::-1, 0].T == -1,
            cbar=True, vmax=65, vmin=-1
        )
        if save_dir is not None:
            plt.savefig(save_dir+'_mv.png')
        plt.show()

    if show_mh:
        # plt.figure(figsize=(10, 36))
        plt.figure(figsize=(20, 10))
        plt.title('mag_horizontal_' + str(num))
        # sns.set(font_scale=0.8)
        # cmap='YlOrRd'
        sns.heatmap(
            data=arr_mv_mh[:, ::-1, 1].T,
            cmap='jet',
            # annot=True, fmt='.0f',
            mask=arr_mv_mh[:, ::-1, 1].T == -1,
            cbar=True, vmax=65, vmin=-1
        )
        if save_dir is not None:
            plt.savefig(save_dir+'_mh.png')
        plt.show()
    return


# 绘制二维坐标图
def paint_xy_list(xy_arr_list, line_label_list, xy_range=None, title=None, save_file=None):
    if xy_range is not None:
        plt.figure(num=1, figsize=((xy_range[1] - xy_range[0]) / 2, (xy_range[3] - xy_range[2]) / 2))
        plt.xlim(xy_range[0], xy_range[1])
        plt.ylim(xy_range[2], xy_range[3])
    # 循环绘制多条线
    if title is not None:
        plt.title(title)
    for xy_arr, line_label in zip(xy_arr_list, line_label_list):
        plt.plot(xy_arr[:, 0], xy_arr[:, 1], label=line_label)
    plt.legend()
    # 保存图片
    if save_file is not None:
        plt.savefig(save_file)
    plt.show()
    plt.close(1)


# 对输入的一维数组进行绘制查看波形
def paint_signal(data_signal, title='data', ylim=60):
    plt.figure(figsize=(3, 7))
    x = range(0, len(data_signal))
    plt.title(label=title, loc='center')
    plt.ylim(0, ylim)
    plt.plot(x, data_signal, label='line', color='g', linewidth=1.0, linestyle='-')
    plt.show()
    plt.close()
    return


# 输入：建图各种参数：图长宽、块大小，绘图轨迹序列(已经栅格化的)，迭代次数，
# 思路：创建和mag_map一样大小的二维空数组，在其中绘图，如果两次序列一样，则不绘图
def paint_iteration_results(map_size_x, map_size_y, block_size, last_xy, new_xy, num):
    # 先根据地图、块大小，计算块的个数，得到数组的长度。向上取整
    shape = [math.ceil(map_size_x / block_size), math.ceil(map_size_y / block_size)]
    map = np.zeros(shape, dtype=float)
    different = False
    for last, new in zip(last_xy, new_xy):
        if last[0] != new[0] or last[1] != new[1]:
            different = True
            break

    if different:
        for new in new_xy:
            map[int(new[0])][int(new[1])] = 1.
        plt.figure(figsize=(19, 10))
        plt.title('Iteration_' + str(num))
        sns.set(font_scale=0.8)
        sns.heatmap(map, cmap='YlOrRd', annot=False)
        plt.show()

    return


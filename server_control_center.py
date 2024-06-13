# 服务器线程控制中心:
# 声明公共数据容器，
# DI到 socket线程、pdr线程、magPdr线程，三个“守护线程”
# 并启动它们
# from server_threads.mag_position_thread import MagPositionThread
from server_threads.mag_position_thread_UWB import MagPositionThread_UWB
from server_threads.pdr_thread import PdrThread
from server_threads.socket_server_thread import SocketServerThread
from mag_and_other_tools.config_tools import SystemConfigurations
from mag_and_other_tools.config_tools import ConfigTypes
import queue
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

def mag_position_server_start(config_json_file):
    # 读取配置文件，将参数封装为配置对象，输入到各个thread对象中
    configurations = SystemConfigurations(config_json_file, ConfigTypes.POSITION)

    # 如果参数读取失败，直接不启动服务器，
    if not configurations.init_succeed:
        print("Read config json file failed! Please stop starting the program and checking the config json file!")
        for inf in configurations.failed_inf:
            print(inf)
        return

    MAP_SIZE_X = configurations.MapSizeX
    MAP_SIZE_Y = configurations.MapSizeY

    socket_output_queue = queue.Queue()
    pdr_input_queue = socket_output_queue
    pdr_output_queue = queue.Queue()
    mag_position_input_queue = pdr_output_queue
    mag_position_output_queue = queue.Queue()

    # TODO 在这里增加接收、存储、传递UWB定位结果的队列，传输到MagPositionThread_UWB中，
    #  如果UWB提供的xy频率太低，则建议插值后再给这边，否则和PDRxy对应误差较大
    #  ***注意提供的UWB xy必须和磁场指纹地图mag_map的坐标系统一！&& 已经补偿了和这边的IMU数据的时间差异
    uwb_input_queue = queue.Queue()  # TODO 提供 list[毫秒时间戳，uwb_x, uwb_y]

    # 定义线程
    socket_server_thread = SocketServerThread(socket_output_queue, configurations)
    pdr_thread = PdrThread(pdr_input_queue, pdr_output_queue, configurations)
    # mag_position_thread = MagPositionThread(mag_position_input_queue, mag_position_output_queue,
    #                                         configurations, socket_server_thread)
    mag_position_thread = MagPositionThread_UWB(mag_position_input_queue, mag_position_output_queue,
                                            configurations, socket_server_thread, uwb_input_queue)
    # 启动线程
    socket_server_thread.start()
    pdr_thread.start()
    mag_position_thread.start()

    # 实时绘制结果图片
    # final_xy = []
    # final_xy.extend(mag_position_output_queue.get())
    # xy_range = [0, MAP_SIZE_X * 1, 0, MAP_SIZE_Y * 1]
    # plt.figure(num=1, figsize=((xy_range[1] - xy_range[0]) / 4, (xy_range[3] - xy_range[2]) / 4))
    # while True:
    #     plt.xlim(xy_range[0], xy_range[1])
    #     plt.ylim(xy_range[2], xy_range[3])
    #     xy_arr = np.array(final_xy)
    #     plt.plot(xy_arr[:, 0], xy_arr[:, 1])
    #     plt.pause(0.5)
    #     cur_data = mag_position_output_queue.get()
    #     if isinstance(cur_data, str) and cur_data == 'END':
    #         break
    #     final_xy.extend(cur_data)
    #     plt.ioff()
    #     plt.clf()
    #
    # print("当前轨迹绘制完毕")
    # plt.close()
    print('inite succeed')


if __name__ == '__main__':
    # 获取控制台参数
    parser = argparse.ArgumentParser()
    parser.add_argument("json", type=str, help="Absolute path of json file.")
    args = parser.parse_args()
    config_json_file = args.json
    mag_position_server_start(config_json_file)

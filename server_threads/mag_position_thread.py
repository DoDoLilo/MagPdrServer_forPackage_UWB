import threading
from enum import Enum
import math
import numpy as np
import mag_and_other_tools.mag_mapping_tools as MMT

# 地磁定位线程，状态枚举类
class MagPositionState(Enum):
    INITIALIZING = 0  # 初始化状态：从BROKEN态转来，正处于初始固定区域遍历
    STABLE_RUNNING = 1  # 平稳运行状态：从INITIALIZING态转来，初始遍历成功，以此基础进行匹配定位
    STOP = 2  # 停止态：由STABLE_RUNNING转来，in_data_queue得到time=-1的主动结束标志


# 地磁定位 守护线程：检查公共数据容器中的数据，并根据数据进行状态转移、定位计算
# 初始态：清空历史数据，当容器中的数据量达到initial_dis则调用初始遍历算法
# 运行态：基于之前的transfer进行计算
class MagPositionThread(threading.Thread):
    def __init__(self, in_data_queue, out_data_queue, configurations):
        super(MagPositionThread, self).__init__()
        # 初始化各种参数
        self.state = MagPositionState.STOP
        self.in_data_queue = in_data_queue
        self.out_data_queue = out_data_queue
        self.out_data_sock_file = configurations.LocalSocketMagposition.makefile(mode='w')

        # 从配置文件中读取各种参数，并赋予成员变量
        # -----------地图系统参数------------------
        self.BLOCK_SIZE = configurations.BlockSize  # 地图块大小（m），必须和使用的指纹库文件建库时的块大小一致
        self.INITAIL_BUFFER_DIS = configurations.InitailBufferDis  # 初始态匹配时，缓存池大小（m） > BUFFER_DIS!
        self.BUFFER_DIS = configurations.BufferDis  # 稳定态匹配时，缓冲池大小（m）
        self.DOWN_SIP_DIS = configurations.DownSipDis  # 下采样粒度（m），应为块大小的整数倍？（下采样越小则相同长度序列的匹配点越多，匹配难度越大！）
        # --------迭代搜索参数----------------------
        self.SLIDE_STEP = configurations. SlideStep  # 滑动窗口步长
        self.SLIDE_BLOCK_SIZE = configurations.SlideBlockSize  # 滑动窗口最小粒度（m），>=DOWN_SIP_DIS！
        self.MAX_ITERATION = configurations.MaxIteration  # 高斯牛顿最大迭代次数
        self.TARGET_MEAN_LOSS = configurations.TargetMeanLoss  # 目标损失
        self.ITER_STEP = configurations.IterStep  # 迭代步长，牛顿高斯迭代是局部最优，步长要小
        self.UPPER_LIMIT_OF_GAUSSNEWTEON = configurations.UpperLimitOfGaussNewteon # 当前参数下高斯牛顿迭代MAX_ITERATION的能降低的loss上限
        # ---------其他参数----------------------------
        self.EMD_FILTER_LEVEL = configurations.EmdFilterLevel  # 低通滤波的程度，值越大滤波越强。整型，无单位。
        self.PDR_IMU_ALIGN_SIZE = configurations.PdrImuAlignSize  # 1个PDR坐标对应的imu\iLocator数据个数，iLocator与imu已对齐
        self.TRANSFERS_PRODUCE_CONFIG = configurations.TransfersProduceConfig  # 枚举transfers的参数，[0] = [△x, △y(米), △angle(弧度)], [1] = [枚举的正负个数]
        # ---------数据文件路径---------------------------
        # 地磁指纹库文件，[0]为mv.csv，[1]为mh.csv
        self.MAG_MAP_FILES = configurations.MagMapFiles
        self.COORDINATE_OFFSET = configurations.CoordinateOffset  # move_x,move_y
        self.ENTRANCE_LIST = configurations.EntranceList

        mag_map = MMT.rebuild_map_from_mvh_files(self.MAG_MAP_FILES)
        if mag_map is None:
            print("地磁指纹文件错误，初始化失败！")
            return
        self.mag_map = mag_map

    def run(self) -> None:
        self.mag_position_thread(self.in_data_queue, self.out_data_queue, self.COORDINATE_OFFSET, self.ENTRANCE_LIST)

    # 输入：容器引用、地图坐标系参数（左下角、右上角坐标）、地图所有入口的坐标
    #    coordinate_offset，将entrance坐标平移到指纹库坐标系0-map_size_x, 0-map_size_y的move_x,move_y
    # 从in_data_queue中获取pdr_thread输出的 [time, [pdr_x, y], [10*[mag x, y, z]]]
    # 往out_data_list中放入[time, [mag_position_x,y]]
    def mag_position_thread(self, in_data_queue, out_data_queue, coordinate_offset, entrance_list) -> None:
        transfer = None
        first_window_start_distance = self.INITAIL_BUFFER_DIS + self.SLIDE_BLOCK_SIZE * self.SLIDE_STEP - self.BUFFER_DIS  # 第一个滑动窗口的起始距离
        window_buffer = []
        window_buffer_dis = 0
        pdr_index_list = []
        slide_distance = self.SLIDE_STEP * self.SLIDE_BLOCK_SIZE

        while True:
            # 根据不同的状态对该数据做对应的处理
            if self.state == MagPositionState.STOP:
                # 重新开始
                self.state = MagPositionState.INITIALIZING

                cur_data = in_data_queue.get()
                if isinstance(cur_data, str) and cur_data == "END":
                    self.state = MagPositionState.STOP
                    continue
                # 重新初始化各种参数
                inital_data_buffer = [cur_data]
                window_buffer = []
                window_buffer_dis = 0
                pdr_index_list = []
                window_start = False
                distance = 0

                # 从数据输入流中取地足够初始遍历的数据。注意如果过程中遇到-1，则回到STOP状态
                while distance < self.INITAIL_BUFFER_DIS:
                    cur_data = in_data_queue.get()
                    if isinstance(cur_data, str) and cur_data == 'END':
                        self.state = MagPositionState.STOP
                        break

                    last_data = inital_data_buffer[len(inital_data_buffer) - 1]
                    inital_data_buffer.append(cur_data)
                    dis_incre = math.hypot(cur_data[1][0] - last_data[1][0], cur_data[1][1] - last_data[1][1])
                    distance += dis_incre

                    if window_start:
                        window_buffer_dis += dis_incre
                    if not window_start and distance >= first_window_start_distance:
                        window_start = True  # 代表可以往滑动窗口中放东西了
                    if window_start:
                        window_buffer.append(cur_data)

                if self.state == MagPositionState.STOP:
                    continue
                # 先要将entrance_list坐标根据coordinate_offset映射到0-map_size_x, 0-map_size_y的坐标系下 → move_x, move_y
                for e in range(0, len(entrance_list)):
                    entrance_list[e][0] += coordinate_offset[0]
                    entrance_list[e][1] += coordinate_offset[1]

                # 初始化搜索成功，状态转移至稳定搜索阶段，*而pdr坐标，并不需要使用move xy!都会包含在transfer里面
                # 预处理pdr发送过来的数据，将[N][time, [pdr_x, y], [10*[mag x, y, z]]]变为[N][x,y, mv, mh]并且下采样
                match_seq, pdr_index_arr = MMT.change_pdr_thread_data_to_match_seq(inital_data_buffer, self.DOWN_SIP_DIS,
                                                                                   self.EMD_FILTER_LEVEL)
                pdr_index_list.extend(pdr_index_arr)

                # 调用初始化固定区域搜索
                inital_transfer, inital_map_xy, inital_loss = MMT.inital_full_deep_search(
                    entrance_list, match_seq,
                    self.mag_map, self.BLOCK_SIZE,
                    self.ITER_STEP, self.MAX_ITERATION, self.UPPER_LIMIT_OF_GAUSSNEWTEON
                )

                if inital_transfer is None:
                    # 怎么办？认为失败了
                    print("初始搜索失败，请检查使用规范！")
                    self.state = MagPositionState.STOP
                    continue

                # 将结果放入输出队列中，注意外部如果不及时取走结果，这一步可能（队列满）会阻塞，导致后续代码全部停止！
                out_data_queue.put(inital_map_xy)
                self.write_data_to_sock_file(inital_map_xy)
                transfer = inital_transfer.copy()
                print("Initial Transfer = ", transfer)
                print("Initial Loss = ", inital_loss)
                self.state = MagPositionState.STABLE_RUNNING
                continue

            if self.state == MagPositionState.STABLE_RUNNING:
                # 稳定运行态
                #  此时 每往window_buffer中放入DOWN_SIP_DIS长度的数据，则调用一次匹配算法，并将结果只应用到新加的那段坐标
                #  最后从window_buffer中删除开头一段长度等于DOWN_SIP_DIS的数据
                while self.state == MagPositionState.STABLE_RUNNING:
                    cur_data = in_data_queue.get()
                    if isinstance(cur_data, str) and cur_data == 'END':
                        self.state = MagPositionState.STOP
                        out_data_queue.put('END')  # 放入-1供外部知晓停止
                        out_data_queue.put(np.array(pdr_index_list))
                        self.write_data_to_sock_file('END')
                        print("当前定位结束")
                        break

                    last_data = window_buffer[len(window_buffer) - 1]
                    window_buffer.append(cur_data)
                    window_buffer_dis += math.hypot(cur_data[1][0] - last_data[1][0], cur_data[1][1] - last_data[1][1])

                    if window_buffer_dis >= self.BUFFER_DIS:
                        # 填满一个窗口，调用一次匹配算法，并将结果只应用到新加的那段坐标
                        print("\n")
                        match_seq, pdr_index_arr = MMT.change_pdr_thread_data_to_match_seq(window_buffer, self.DOWN_SIP_DIS,
                                                                                           self.EMD_FILTER_LEVEL)
                        # 只往pdr_index_list放入比末尾大的新下标
                        max_index = pdr_index_list[len(pdr_index_list) - 1]
                        new_xy_num = 0
                        for pi in range(0, len(pdr_index_arr)):
                            if pdr_index_arr[pi] > max_index:
                                pdr_index_list.extend(pdr_index_arr[pi:])
                                new_xy_num = len(pdr_index_arr) - pi
                                break

                        start_transfer = transfer.copy()
                        transfer, map_xy = MMT.produce_transfer_candidates_and_search(start_transfer,
                                                                                      self.TRANSFERS_PRODUCE_CONFIG,
                                                                                      match_seq, self.mag_map,
                                                                                      self.BLOCK_SIZE, self.ITER_STEP, self.MAX_ITERATION,
                                                                                      self.TARGET_MEAN_LOSS,
                                                                                      self.UPPER_LIMIT_OF_GAUSSNEWTEON,
                                                                                      MMT.SearchPattern.BREAKE_ADVANCED_AND_USE_SECOND_LOSS_WHEN_FAILED)

                        print("transfer = ", transfer)
                        print("window points number = ", len(window_buffer))
                        print("window buffer dis = ", window_buffer_dis)
                        # 只取出map_xy中slide_distance长度的结果，放入到out_data_queue中
                        new_xy = map_xy[len(map_xy) - new_xy_num: len(map_xy), :]
                        out_data_queue.put(new_xy)
                        self.write_data_to_sock_file(new_xy)

                        # 从window_buffer中舍弃开头slide_distance长度的数据
                        abandon_index = len(window_buffer)
                        abandon_dis = 0
                        for ai in range(1, len(window_buffer)):
                            abandon_dis += math.hypot(window_buffer[ai][1][0] - window_buffer[ai - 1][1][0],
                                                      window_buffer[ai][1][1] - window_buffer[ai - 1][1][1])
                            if abandon_dis >= slide_distance:
                                abandon_index = ai
                                break

                        del window_buffer[0: abandon_index]  # 删除窗口头部滑动数据，代表滑动窗口
                        window_buffer_dis -= abandon_dis

                continue

    # 将数据（字符串or xy_list）转为对应形式写到socket文件流中
    def write_data_to_sock_file(self, data):
        if data is None:
            return
        if isinstance(data, list) or isinstance(data, np.ndarray):
            # 要发送的是x,y坐标数据
            for xy in data:
                self.out_data_sock_file.write(str(xy[0]) + ',' + str(xy[1]) + '\n')
            self.out_data_sock_file.flush()
        if isinstance(data, str):
            # 要发送的是字符数据
            self.out_data_sock_file.write(data + '\n')
            self.out_data_sock_file.flush()
        else:
            return

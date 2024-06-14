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
class MagPositionThread_UWB(threading.Thread):
    # TODO 返回的是弧度，不是角度
    def two_slope_angle_off(self, v1, v2):
        # 方向向量
        x1, y1 = v1
        x2, y2 = v2
        det = x1 * y2 - y1 * x2
        dot = x1 * x2 + y1 * y2
        theta = np.arctan2(det, dot)
        theta = theta if theta > 0 else 2 * np.pi + theta
        return theta
    def get_startXyList_and_angleRange_by_UWBxy(self, start_pdr_xy, end_pdr_xy,start_uwb_xy, end_uwb_xy):
        # TODO，增加UWB坐标信息，但不编写如何接收UWB坐标的逻辑，这个让他们集成、or编写，
        #  增加坐标转换功能，获取滑窗首尾UWB坐标，起点构造一定BFS范围后，作为entrances_list，
        #  然后计算首尾坐标和PDR首尾坐标的夹角，以该角度为中心、设定inital_full_search的搜索角度范围（注意角度是顺时针还是逆时针）
        # adjust_trajs_by_marks_2.py中的 计算两直线的 始末点向量 夹角
        vector_pdr = end_pdr_xy[0] - start_pdr_xy[0], end_pdr_xy[1] - start_pdr_xy[1]
        vector_uwb = end_uwb_xy[0] - start_uwb_xy[0], end_uwb_xy[1] - start_uwb_xy[1]
        angle_off = self.two_slope_angle_off(vector_pdr, vector_uwb)
        # TODO angle_off就是transfer中的angle（直接用），用它构造angle range
        move_config = [[0.3, 0.3], [2, 2]]  # 枚举间隔、正负数量，类似TransfersProduceConfig
        entrance_list = MMT.produce_entrance_candidates_ascending(start_uwb_xy, move_config) # 为什么？因为UWBxy存在一定误差范围
        angle_range = [math.degrees(angle_off)-22, math.degrees(angle_off)+22]  # TODO 注意这里使用的是角度，不是弧度
        return entrance_list, angle_range

    def __init__(self, in_data_queue, out_data_queue, configurations, socket_server_thread, in_uwbXy_queue):
        super(MagPositionThread_UWB, self).__init__()
        # 初始化各种参数
        self.state = MagPositionState.STOP
        self.in_data_queue = in_data_queue
        self.out_data_queue = out_data_queue
        # self.out_data_sock_file = configurations.LocalSocketMagposition.makefile(mode='w')
        self.socket_server_thread = socket_server_thread
        # +UWB, list[毫秒时间戳，uwb_x, uwb_y]
        self.in_uwbXy_queue = in_uwbXy_queue

        # 从配置文件中读取各种参数，并赋予成员变量
        # -----------定位结果文件保存路径------------------------
        self.position_save_dir = configurations.PositionSaveDir
        # -----------地图系统参数------------------
        self.BLOCK_SIZE = configurations.BlockSize  # 地图块大小（m），必须和使用的指纹库文件建库时的块大小一致
        self.INITAIL_BUFFER_DIS = configurations.InitailBufferDis  # 初始态匹配时，缓存池大小（m） > BUFFER_DIS!
        self.BUFFER_DIS = configurations.BufferDis  # 稳定态匹配时，缓冲池大小（m）
        self.DOWN_SIP_DIS = configurations.DownSipDis  # 下采样粒度（m），应为块大小的整数倍？（下采样越小则相同长度序列的匹配点越多，匹配难度越大！）
        # --------迭代搜索参数----------------------
        self.SLIDE_STEP = configurations.SlideStep  # 滑动窗口步长
        self.SLIDE_BLOCK_SIZE = configurations.SlideBlockSize  # 滑动窗口最小粒度（m），>=DOWN_SIP_DIS！
        # self.MAX_ITERATION = configurations.MaxIteration  # 高斯牛顿最大迭代次数
        self.MAX_ITERATION = 5  # 高斯牛顿最大迭代次数
        self.TARGET_MEAN_LOSS = configurations.TargetMeanLoss  # 目标损失
        self.ITER_STEP = configurations.IterStep  # 迭代步长，牛顿高斯迭代是局部最优，步长要小
        # self.UPPER_LIMIT_OF_GAUSSNEWTEON = configurations.UpperLimitOfGaussNewteon  # 当前参数下高斯牛顿迭代MAX_ITERATION的能降低的loss上限
        self.UPPER_LIMIT_OF_GAUSSNEWTEON = 0.1  # 当前参数下高斯牛顿迭代MAX_ITERATION的能降低的loss上限
        # ---------其他参数----------------------------
        self.EMD_FILTER_LEVEL = configurations.EmdFilterLevel  # 低通滤波的程度，值越大滤波越强。整型，无单位。
        self.PDR_IMU_ALIGN_SIZE = configurations.PdrImuAlignSize  # 1个PDR坐标对应的imu\iLocator数据个数，iLocator与imu已对齐
        self.TRANSFERS_PRODUCE_CONFIG = configurations.TransfersProduceConfig  # 枚举transfers的参数，[0] = [△x, △y(米), △angle(弧度)], [1] = [枚举的正负个数]
        # ---------数据文件路径---------------------------
        # 地磁指纹库文件，[0]为mv.csv，[1]为mh.csv
        self.MAG_MAP_FILES = configurations.MagMapFiles
        self.COORDINATE_OFFSET = configurations.CoordinateOffset  # move_x,move_y
        self.ENTRANCE_LIST = configurations.EntranceList
        self.MOVE_X = configurations.MoveX
        self.MOVE_Y = configurations.MoveY

        # 先要将entrance_list坐标根据coordinate_offset映射到0-map_size_x, 0-map_size_y的坐标系下 → move_x, move_y
        # 注意提前在这里做了，而不是在mag_position_thread里每次都做，导致重复平移entrance_list、第一次后都错了
        for e in range(0, len(self.ENTRANCE_LIST)):
            self.ENTRANCE_LIST[e][0] += self.COORDINATE_OFFSET[0]
            self.ENTRANCE_LIST[e][1] += self.COORDINATE_OFFSET[1]

        mag_map = MMT.rebuild_map_from_mvh_files(self.MAG_MAP_FILES)
        if mag_map is None:
            print("地磁指纹文件错误，初始化失败！")
            return
        self.mag_map = mag_map

    def run(self) -> None:
        self.mag_position_thread(self.in_data_queue, self.out_data_queue, self.ENTRANCE_LIST)

    # 输入：容器引用、地图坐标系参数（左下角、右上角坐标）、地图所有被平移到指纹库坐标系的入口坐标
    # 从in_data_queue中获取pdr_thread输出的 [time, [pdr_x, y], [10*[mag x, y, z, quat x, y, z, w]], pdr_index]
    # 往out_data_list中放入[time, [mag_position_x,y]]
    def mag_position_thread(self, in_data_queue, out_data_queue, entrance_list) -> None:
        transfer = None
        first_window_start_distance = self.INITAIL_BUFFER_DIS + self.SLIDE_BLOCK_SIZE * self.SLIDE_STEP - self.BUFFER_DIS  # 第一个滑动窗口的起始距离
        window_buffer = []
        window_buffer_dis = 0
        pdr_index_list = []
        slide_distance = self.SLIDE_STEP * self.SLIDE_BLOCK_SIZE

        while True:
            print("state is:", self.state)
            # 根据不同的状态对该数据做对应的处理
            # ==========================================================================================================
            # -------------------------初始化状态------------------------------------------------------------------------
            if self.state == MagPositionState.STOP:
                # 重新开始
                self.state = MagPositionState.INITIALIZING

                cur_data = in_data_queue.get()  # 阻塞直到有PDR与IMU数据来
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
                # TODO 从 self.in_uwbXy_queue中获取第一个uwb_xy，queue.get()默认阻塞直到获得uwb_xy。获取不到，则一直阻塞？No，设定最大时间（3秒）
                #  根据“UWBS”丢弃其前面可能留下的上一次定位的、没使用的UWB坐标。
                find_UWBS = False
                while not find_UWBS:
                    try:
                        uwb_data = self.in_uwbXy_queue.get(block=True, timeout=3)
                        if isinstance(uwb_data, str) and cur_data == 'UWBS':
                            find_UWBS = True
                    except Exception as e:
                        print("No UWB xy in 3 seconds")
                        self.state = MagPositionState.STOP
                if self.state == MagPositionState.STOP:
                    continue

                # TODO 获取本次定位最开始的UWB坐标
                try:
                    start_uwb_pos = self.in_uwbXy_queue.get(block=True, timeout=3)  # list[毫秒时间戳，uwb_x, uwb_y]
                    start_uwb_time = start_uwb_pos[0]
                except Exception as e:
                    print("No UWB xy in 3 seconds")
                    self.state = MagPositionState.STOP
                    continue
                if self.state == MagPositionState.STOP:
                    continue

                # 从数据输入流中取地足够初始遍历的数据。注意如果过程中遇到END，则回到STOP状态
                while distance < self.INITAIL_BUFFER_DIS:
                    cur_data = in_data_queue.get()  # [time, [pdr_x, y], [10*[mag x, y, z, quat x, y, z, w]], pdr_index]
                    if isinstance(cur_data, str) and cur_data == 'END':
                        self.state = MagPositionState.STOP
                        print("END recived.")
                        # print("state is:", self.state)
                        break

                    # TODO 将cur_data.time早于start_uwb_time的所有数据舍弃
                    cur_time = cur_data[0]
                    if cur_time < start_uwb_time:
                        continue  # 继续循环

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

                # TODO 初始窗口填补完毕 + 已有窗口起点UWB坐标
                #  此时需要获取窗口末尾UWB坐标（从self.in_uwbXy_queue中获得第一个时间戳大于磁场坐标的UWB数据），获取不到，则一直阻塞？No，设定最大时间3秒。
                find_end_uwb_xy = False
                while not find_end_uwb_xy:
                    try:
                        end_uwb_pos = self.in_uwbXy_queue.get(block=True, timeout=5)  # list[毫秒时间戳，uwb_x, uwb_y]
                        end_uwb_time = end_uwb_pos[0]
                        if end_uwb_time >= window_buffer[-2][0]:
                            find_end_uwb_xy = True
                    except Exception as e:
                        print("No UWB xy in 3 seconds")
                        self.state = MagPositionState.STOP
                        # 初始化数据准备失败！
                        break

                # TODO 注意跟上面一样加这个，否则会继续初始化
                if self.state == MagPositionState.STOP:
                    continue

                start_pdr_xy = [window_buffer[0][1][0], window_buffer[0][1][1]]  # TODO 数组引用精确到最后一位，否则为地址，可能出现问题。
                end_pdr_xy = [window_buffer[-1][1][0], window_buffer[-1][1][1]]
                start_uwb_xy = [start_uwb_pos[1], start_uwb_pos[2]]
                end_uwb_xy = [end_uwb_pos[1], end_uwb_pos[2]]
                # TODO 准备好了首尾UWB xy，计算PDR和UWB夹角 -> 给出大概的搜索角范围，
                #  start_uwb_xy替换为entrance_list中的entrance[x,y]，执行inital_full_deep_search_with_angleRange
                entrance_list, angle_range = self.get_startXyList_and_angleRange_by_UWBxy(start_pdr_xy, end_pdr_xy, start_uwb_xy, end_uwb_xy)

                # 初始化搜索成功，状态转移至稳定搜索阶段，*而pdr坐标，并不需要使用move xy!都会包含在transfer里面
                # 预处理pdr发送过来的数据，将[N][time, [pdr_x, y], [10*[mag x, y, z, quat x, y, z, w]], pdr_index]变为
                # 下采样后的[N][x,y, mv, mh] [N][pdr_index] [N][xy_time]
                match_seq, pdr_index_arr, time_arr = MMT.change_pdr_thread_data_to_match_seq(inital_data_buffer,
                                                                                             self.DOWN_SIP_DIS,
                                                                                             self.EMD_FILTER_LEVEL)
                pdr_index_list.extend(pdr_index_arr)

                # 调用初始化固定区域搜索
                # inital_transfer, inital_map_xy, inital_loss = MMT.inital_full_deep_search(
                #     entrance_list, match_seq,
                #     self.mag_map, self.BLOCK_SIZE,
                #     self.ITER_STEP, self.MAX_ITERATION, self.TARGET_MEAN_LOSS
                # )
                # TODO 增加angleRange
                inital_transfer, inital_map_xy, inital_loss = MMT.inital_full_deep_search_with_angleRange(
                    entrance_list, angle_range,match_seq,
                    self.mag_map, self.BLOCK_SIZE,
                    self.ITER_STEP, self.MAX_ITERATION, self.TARGET_MEAN_LOSS
                )

                if inital_transfer is None:
                    # 怎么办？认为失败了
                    print("初始搜索失败，请确保指纹库建立正确、覆盖面积足够，or 减少初始窗口大小（但要保证比滑动窗口大）。")
                    self.state = MagPositionState.STOP
                    # print("state is:", self.state)
                    continue

                # 将结果放入输出队列中，注意外部如果不及时取走结果，这一步可能（队列满）会阻塞，导致后续代码全部停止！
                # out_data_queue.put(inital_map_xy)
                # self.write_data_to_sock_file(self.add_head_to_xy(inital_map_xy))
                transfer = inital_transfer.copy()
                self.state = MagPositionState.STABLE_RUNNING

                # 记录结果
                position_save_arr = self.add_head_to_xy(inital_map_xy, time_arr)
                # self.print_mag_position_msg(self.add_head_to_xy(inital_map_xy, time_arr))
                print("Initial Transfer = ", transfer)
                print("Initial Loss = ", inital_loss)
                continue


            # =========================================================================================================
            # --------------------------------初始化后的稳定运行态--------------------------------------------------------
            # TODO 在运行态时RUNNING，要不断检测UWB_xy_queue是否满足条件，计算新的transfer（不切换回STOP状态，否则滑窗数据要丢）
            if self.state == MagPositionState.STABLE_RUNNING:
                # 稳定运行态
                #  此时 每往window_buffer中放入DOWN_SIP_DIS长度的数据，则调用一次匹配算法，并将结果只应用到新加的那段坐标
                #  最后从window_buffer中删除开头一段长度等于DOWN_SIP_DIS的数据
                while self.state == MagPositionState.STABLE_RUNNING:
                    cur_data = in_data_queue.get()
                    if isinstance(cur_data, str) and cur_data == 'END':
                        self.state = MagPositionState.STOP
                        # 根据手机号、第一帧的时间戳构建文件名，保存至指定文件夹下
                        user_phone = self.socket_server_thread.user_phone
                        user_phone = '000' if user_phone is None or user_phone == '' else str(int(user_phone))
                        start_time = str(int(position_save_arr[0][0]))
                        file_name = self.position_save_dir + '/' + user_phone + "_" + start_time + ".csv"
                        np.savetxt(file_name, position_save_arr, delimiter=',', fmt='%.2f')
                        print("User " + user_phone + " end position, save to: " + file_name)

                        # out_data_queue.put('END')
                        # out_data_queue.put(np.array(pdr_index_list))
                        # self.write_data_to_sock_file('END')
                        print('END')
                        break

                    last_data = window_buffer[len(window_buffer) - 1]
                    window_buffer.append(cur_data)
                    window_buffer_dis += math.hypot(cur_data[1][0] - last_data[1][0], cur_data[1][1] - last_data[1][1])

                    if window_buffer_dis >= self.BUFFER_DIS:
                        # 填满一个窗口，调用一次匹配算法，并将结果只应用到新加的那段坐标
                        match_seq, pdr_index_arr, time_arr = MMT.change_pdr_thread_data_to_match_seq(window_buffer,
                                                                                           self.DOWN_SIP_DIS,
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

                        # TODO 滑窗前后有对应的UWB xy就用，否则就不用；注意此时取UWBxy的block时间要短，而且抛异常也不影响运行、状态变化
                        find_start_uwb_xy = False
                        while not find_start_uwb_xy:
                            try:
                                start_uwb_pos = self.in_uwbXy_queue.get(block=True, timeout=0.5)
                                start_uwb_time = start_uwb_pos[0]
                                if start_uwb_time >= window_buffer[0][0]:
                                    find_start_uwb_xy = True
                            except Exception as e:
                                break

                        find_end_uwb_xy = False
                        while not find_end_uwb_xy:
                            try:
                                end_uwb_pos = self.in_uwbXy_queue.get(block=True, timeout=0.5)  # list[毫秒时间戳，uwb_x, uwb_y]
                                end_uwb_time = end_uwb_pos[0]
                                if end_uwb_time >= window_buffer[-2][0]:
                                    find_end_uwb_xy = True
                            except Exception as e:
                                break

                        if find_start_uwb_xy and find_end_uwb_xy:
                            # TODO 找到了首尾uwb坐标，那就根据这个重置start_transfer
                            start_pdr_xy = [window_buffer[0][1][0], window_buffer[0][1][1]]  # TODO 数组引用精确到最后一位，否则为地址，可能出现问题。
                            end_pdr_xy = [window_buffer[-1][1][0], window_buffer[-1][1][1]]
                            start_uwb_xy = [start_uwb_pos[1], start_uwb_pos[2]]
                            end_uwb_xy = [end_uwb_pos[1], end_uwb_pos[2]]
                            # TODO start_transfer的偏移量
                            start_transfer[0] = start_uwb_xy[0]-start_pdr_xy[0]
                            start_transfer[1] = start_uwb_xy[1]-start_pdr_xy[1]
                            # TODO start_transfer的旋转量，夹角
                            vector_pdr = end_pdr_xy[0] - start_pdr_xy[0], end_pdr_xy[1] - start_pdr_xy[1]
                            vector_uwb = end_uwb_xy[0] - start_uwb_xy[0], end_uwb_xy[1] - start_uwb_xy[1]
                            start_transfer[2] = self.two_slope_angle_off(vector_pdr, vector_uwb)  # TODO transfer使用的是弧度，不是角度

                        transfer, map_xy = MMT.produce_transfer_candidates_and_search(start_transfer,
                                                                                      self.TRANSFERS_PRODUCE_CONFIG,
                                                                                      match_seq, self.mag_map,
                                                                                      self.BLOCK_SIZE, self.ITER_STEP,
                                                                                      self.MAX_ITERATION,
                                                                                      self.TARGET_MEAN_LOSS,
                                                                                      self.UPPER_LIMIT_OF_GAUSSNEWTEON,
                                                                                      MMT.SearchPattern.BREAKE_ADVANCED_AND_USE_SECOND_LOSS_WHEN_FAILED)

                        # 只取出map_xy中slide_distance长度的结果，放入到out_data_queue中
                        new_xy = map_xy[len(map_xy) - new_xy_num: len(map_xy), :]
                        # out_data_queue.put(new_xy)
                        # self.write_data_to_sock_file(self.add_head_to_xy(new_xy))
                        # 记录地磁定位结果
                        position_save_arr = np.vstack((position_save_arr, self.add_head_to_xy(new_xy, time_arr[len(time_arr)-len(new_xy):len(time_arr)])))
                        # self.print_mag_position_msg(self.add_head_to_xy(new_xy, time_arr))
                        # print("transfer = ", transfer)
                        # print("window points number = ", len(window_buffer))
                        # print("window buffer dis = ", window_buffer_dis)

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
    # def write_data_to_sock_file(self, data):
    #     if data is None:
    #         return
    #     if isinstance(data, list) or isinstance(data, np.ndarray):
    #         # 要发送的是time, user_id, x, y坐标数据
    #         for row in data:
    #             # 将row转为csv的格式后存储
    #             csv_str = ''
    #             for i in range(0, len(row) - 1):
    #                 csv_str += (str(row[i]) + ',') if i != 0 else (str(int(row[i])) + ',')
    #             self.out_data_sock_file.write(csv_str + str(row[len(row) - 1]) + '\n')
    #         self.out_data_sock_file.flush()
    #     if isinstance(data, str):
    #         # 要发送的是字符数据
    #         self.out_data_sock_file.write(data + '\n')
    #         self.out_data_sock_file.flush()
    #     else:
    #         return

    # 为将要发送到socket IO流中的xy坐标数据添加上表明信息的数据，变成 time, use_id, x, y
    def add_head_to_xy(self, new_xy, xy_time):
        # 将new_xy的每一行数据都加上信息头，变成 time, use_id, x, y
        sent_data = np.empty(shape=(len(new_xy), 4))
        for i in range(0, len(new_xy)):
            sent_data[i][0] = xy_time[i]  # 系统时间毫秒
            sent_data[i][1] = self.socket_server_thread.user_phone  # 用户id（手机号）
            sent_data[i][2] = new_xy[i][0] - self.MOVE_X  # 坐标x
            sent_data[i][3] = new_xy[i][1] - self.MOVE_Y  # 坐标y

        return sent_data

    # time, use_id, x, y
    def print_mag_position_msg(self, data):
        for row in data:
            # 将row转为csv的格式后存储
            csv_str = ''
            for i in range(0, len(row) - 1):
                csv_str += (str(row[i]) + ',') if i != 0 else (str(int(row[i])) + ',')
            print('MAG_POSITION:' + csv_str + str(row[len(row) - 1]))

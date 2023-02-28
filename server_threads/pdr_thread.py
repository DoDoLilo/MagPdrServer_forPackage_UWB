import threading
import enum
import pdr_and_model.pdr_vxy as PDR
import math


class PdrState(enum.Enum):
    STOP = 0  # 初始状态，置Px,Py = 0, 0，清空window_buffer
    RUNNING = 1  # 运行状态，当接收到time = -1时，转换置STOP状态


class PdrThread(threading.Thread):
    # in_data_queue 每个单位为
    def __init__(self, in_data_queue, out_data_queue, configurations):
        super(PdrThread, self).__init__()
        self.WALKING_SPEED = 0.3  # 速度低于WALKING_SPEED（m/s）就认为是静止了

        self.in_data_queue = in_data_queue
        self.out_data_queue = out_data_queue
        # self.out_data_sock_file = configurations.LocalSocketPdr.makefile(mode='w')

        self.WINDOW_SIZE = configurations.PdrWindowSize
        self.SLIDE_SIZE = configurations.PdrSlideSize
        self.PDR_MODEL_PATH = configurations.PdrModelFile

        self.state = PdrState.STOP

    def run(self) -> None:
        self.pdr_thread()

    # 接收从socket线程发送过来的数据 time, acc_xyz, gyro_xyz, mag_xyz, gamerotaion_xyzw
    # 滑动窗口大小 = 200，滑动距离 = 10
    # 如果接收到的time = -1，则清空坐标Px,Py，重新计算
    # 模型返回的是Vx,Vy，需要外部记录时间戳进行位置Px,Py计算
    def pdr_thread(self) -> None:
        window_buffer = []
        px, py = 0, 0
        window_size = self.WINDOW_SIZE
        slide_size = self.SLIDE_SIZE
        pdr_index = 0

        while True:
            if self.state == PdrState.STOP:
                window_buffer = []
                px, py = 0, 0
                pdr_index = 0
                # 接收 window_size 个数据，过程中遇到 'END'则不继续
                succeed = True
                for i in range(0, window_size):
                    cur_data = self.in_data_queue.get()
                    if isinstance(cur_data, str) and cur_data == "END":
                        succeed = False
                        break
                    else:
                        window_buffer.append(cur_data)

                # 成功接收满一个window的数据，则进入到运行态
                if succeed:
                    self.state = PdrState.RUNNING
                continue

            if self.state == PdrState.RUNNING:
                # 调用模型计算window_buffer数据的输出Vx,Vy，
                # ΔT = (window_buffer.time[slide_size] - window_buffer.time[0]) / 1000  因为time是ms
                # Px,Py = Vx * ΔT, Vy * ΔT
                # 构建pdr输入数据[200][acc, gyro, ori]
                pdr_input = []
                for line in window_buffer:
                    pdr_input.append([line[1], line[2], line[3],  # ACCELEROMETER
                                      line[4], line[5], line[6],  # GYROSCOPE
                                      line[10], line[11], line[12], line[13]])  # GAME_ROTATION_VECTOR
                # 输入拷贝的窗口数据到pdr模型中估计用户速度
                v_xy = PDR.workpart(pdr_input, self.PDR_MODEL_PATH)
                vx, vy = v_xy[0][0], v_xy[0][1]
                slide_time = (window_buffer[slide_size][0] - window_buffer[0][0]) / 1000
                px += vx * slide_time
                py += vy * slide_time

                # 判断是否停止走路
                if self.is_stop_move(vx, vy):
                    print("PDR_STATE:" + str(int(window_buffer[slide_size][0])) + ',' + 'STANDING' + '\n')
                    # self.out_data_sock_file.write('STANDING' + '\n')
                    # self.out_data_sock_file.flush()

                # 将 [time, [px, py], mag_quat_list, pdr_index]放入out_data_queue中
                # IMU data[ (pdr_i + 1)*10 - 5 : (pdr_i + 1)*10 + 5]
                mag_quat_list = []
                for line in window_buffer[5: slide_size + 5]:
                    mag_quat_list.append([line[7], line[8], line[9],
                                          line[10], line[11], line[12], line[13]])

                self.out_data_queue.put([window_buffer[slide_size][0],
                                         [px, py],
                                         mag_quat_list,
                                         pdr_index])
                pdr_index += 1

                # 继续接收 slide_size 个数据，过程中遇到'END'，不仅要转换状态，还要往out_data_queue中放入'END'让后续线程知晓
                del window_buffer[0: slide_size]  # 删除开头，滑动窗口
                for i in range(0, slide_size):
                    cur_data = self.in_data_queue.get()
                    if isinstance(cur_data, str) and cur_data == "END":
                        self.state = PdrState.STOP
                        self.out_data_queue.put("END")
                        break
                    else:
                        window_buffer.append(cur_data)
                continue

    def is_stop_move(self, vx, vy):
        # 根据pdr输出的速度推测用户是否停在原地不动
        speed = math.hypot(vx, vy)
        if speed < self.WALKING_SPEED:
            return True
        else:
            return False

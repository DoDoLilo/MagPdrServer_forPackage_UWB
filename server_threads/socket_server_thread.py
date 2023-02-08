import threading
import socket


# socket server 守护线程
# 往out_data_list中放入：time, acc, gyro, mag, ori
# 1、如何解决 连上客户端后，客户端与网络断开(客户端自己会知晓，然后不断重复连接)，但本服务端却无法知晓，该socket连接仍占用着
#       解决办法：允许连接多个socket，但只允许存活最新的socket。当新接受一个socket时，如果存在一个旧socket，则主动断开它！
#       还要在断开旧socket时，往out_data_queue中放入END标志，重新开始定位。
#
# 2、socket server thread中的逻辑要改，不能在APP断开重连时仍要读取手机号，
#   且删除readline=“”就切换为stop状态重新开始的逻辑，此时APP可能在尝试重连，
#   而非离开房间。APP端在destroy中会调用leavingroom发送end，
#   但如果连接失败这个end都发送不到服务器。
#   所以服务端需要增加一个以手机号为起始标志的判断，和end一起做完结束running状态的标志，起到当end没有时的作用。
class SocketServerThread(threading.Thread):
    def __init__(self, out_data_queue, configurations):
        super(SocketServerThread, self).__init__()
        self.out_data_queue = out_data_queue  # 输出数据的队列
        self.server_port = configurations.ServerPort  # 监听的端口号
        self.user_phone = None  # 最后定位的用户身份（手机号）
        self.cur_socket_connection = None  # 保存当前正在连接的socket连接
        self.cur_socket_file = None  # 用当前socket连接创建的文件流

    def run(self) -> None:
        self.socket_server_thread()

    def socket_server_thread(self):
        # socket服务器端，一直在监听，同一时刻只允许连接一个socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(self.server_port)
        s.bind(('0.0.0.0', self.server_port))  # 监听端口
        s.listen()

        while True:
            print('Waiting for connection...Port = ', self.server_port)
            sock, addr = s.accept()  # 建立socket连接，但是同一时刻只允许连接一个socket

            # 在存在连接的情况下来了新的连接，需要主动让对应的数据接受线程退出
            #   这里我们在外面主动对IO流/socket连接进行关闭，里面的readline()会异常然后退出，
            #   达到了停止对应的数据接受线程的目的
            if self.cur_socket_file is not None:
                self.cur_socket_file.close()
                self.cur_socket_file = None
            if self.cur_socket_connection is not None:
                self.cur_socket_connection.close()
                self.cur_socket_connection = None
                print('Connection closed.')

            # 开启接受数据子线程
            self.cur_socket_connection = sock
            self.cur_socket_file = sock.makefile(mode='r')
            print('Accept new connection from %s:%s...' % addr)
            t = threading.Thread(target=self.socket_data_reciver)
            t.start()

        return

    def socket_data_reciver(self):
        is_first_line = True
        while True:
            try:
                line = self.cur_socket_file.readline()

                if line == '':
                    # 数据为空，客户端断开连接，但不代表此次定位结束
                    break

                if line == "END\n" or line == "END":
                    # 接受到定位接受标志，客户端定位结束
                    self.out_data_queue.put('END')
                    break

                if is_first_line:
                    #  因为手机端发送手机号的逻辑只会发送在第一次建立socket连接时
                    #  判断这个新建立的socket连接的第一行数据是不是用户手机号，如果不是，则是接着上一次的定位
                    is_first_line = False
                    if self.is_user_id(line):
                        # 是手机号，则要往out_data_queue中放入END，以应对手机端onDestory()结束时未能发出END的情况！
                        self.out_data_queue.put("END")
                        self.user_phone = int(line)
                        print("user phone = ", self.user_phone)
                        continue

                # 第一行不是手机号，就是接着上次的IMU数据
                str_list = line.split(',')
                float_list = [int(float(str_list[0]))]
                for s in str_list[1:]:
                    float_list.append(float(s))
                self.out_data_queue.put(float_list)
            except Exception as e:
                print(e)
                break

        # 服务端断开连接
        if self.cur_socket_file is not None:
            self.cur_socket_file.close()
            self.cur_socket_file = None
        if self.cur_socket_connection is not None:
            self.cur_socket_connection.close()
            self.cur_socket_connection = None
            print('Connection closed.')

    # 该函数判断输入的str是否是用户的身份信息
    # 目前的身份信息的规则很简单，只要不是imu多列数据的，就是身份信息
    def is_user_id(self, line):
        try:
            str_line = str(line)
            if str_line is None or str_line == "":
                return False
            if len(str_line.split(',')) != 1:
                return False
        except Exception as e:
            return False

        return True

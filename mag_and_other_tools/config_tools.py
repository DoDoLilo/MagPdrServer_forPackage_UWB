import json
from collections import OrderedDict
import math
import os
import socket
from enum import Enum


class ConfigTypes(Enum):
    MAP_BUILD = 0
    POSITION = 1


# 配置文件对象
class SystemConfigurations():

    def __init__(self, config_json_file, config_type):
        self.config_json_file = config_json_file
        #  注意有些参数配置文件中没有（DownSipDis）、有些参数读出来后需要转换（TransfersProduceConfig中的角度变成弧度，UpperLimitOfGaussNewteon）
        #  如果这一步都出错，则抛出异常，不进行后续操作
        # TODO 检查设置的参数是否异常
        self.failed_inf = []
        self.init_succeed = True
        # self.LocalSocketMagposition = None
        # self.LocalSocketPdr = None

        try:
            json_str = ''
            with open(config_json_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.split('//')[0] + '\n'
                    json_str += line
            para_dict = json.loads(json_str, object_pairs_hook=OrderedDict)

            # 配置文件所有参数，并检查参数是否异常
            #   地磁指纹地图坐标系大小 0-MAP_SIZE_X ，0-MAP_SIZE_Y（m）（建库、指示绘图时使用）
            self.MapSizeX = para_dict['MapSizeX']
            self.MapSizeY = para_dict['MapSizeY']
            if self.MapSizeX <= 0 or self.MapSizeY <= 0:
                self.init_succeed = False
                self.failed_inf.append('MapSizeX <= 0 or MapSizeY <= 0')

            #   对iLocator_xy平移到地磁指纹低地图坐标系下（在使用iLocator进行建库时使用）
            self.MoveX = para_dict['MoveX']
            self.MoveY = para_dict['MoveY']

            #   地图地磁块大小
            self.BlockSize = para_dict['BlockSize']
            # 低通滤波的程度，值越大滤波越强。整型，无单位。
            self.EmdFilterLevel = para_dict['EmdFilterLevel']
            #   建库时的内插半径，单位（m）
            self.InterRadius = para_dict['InterRadius']
            #   是否删除多余内插块，1代表是，0代表不
            self.DeleteExtraBlocks = True if para_dict['DeleteExtraBlocks'] == 1 else False
            #   删除多余内插块的程度，越大删除的内插范围越大，可以为负值
            self.DeleteLevel = para_dict['DeleteLevel']

            #   如果是定位阶段，则继续读取更多参数并初始化
            if config_type is ConfigTypes.POSITION:
                #   神经网络pdr模型文件存放位置、pdr窗口大小、pdr窗口滑动距离
                # TODO 到时候应该是将模型也打包封装不让外部看到，这个属性可能不需要
                self.PdrModelFile = para_dict['PdrModelFile']
                if not self.PdrModelFile.endswith('.pt') or not os.path.exists(self.PdrModelFile):
                    self.init_succeed = False
                    self.failed_inf.append('PdrModelFile没有以.pt结尾 or PdrModelFile文件不存在')
                self.PdrWindowSize = para_dict['PdrWindowSize']
                self.PdrSlideSize = para_dict['PdrSlideSize']
                self.PdrImuAlignSize = self.PdrSlideSize
                if self.PdrWindowSize <= 0 or self.PdrSlideSize <= 0 or self.PdrSlideSize >= self.PdrWindowSize:
                    self.init_succeed = False
                    self.failed_inf.append('PdrWindowSize <= 0 or PdrSlideSize <= 0 or PdrSlideSize >= PdrWindowSize')

                # TODO 将BlockeSize、MapSizeXY在mag map build save时写到csv文件中，以避免手工设置！
                self.DownSipDis = self.BlockSize
                self.InitailBufferDis = para_dict['InitailBufferDis']
                self.BufferDis = para_dict['BufferDis']
                if self.InitailBufferDis <= 0 or self.BufferDis <= 0 or self.BufferDis >= self.InitailBufferDis:
                    self.init_succeed = False
                    self.failed_inf.append('InitailBufferDis <= 0 or BufferDis <= 0 or BufferDis >= InitailBufferDis')

                self.SlideStep = para_dict['SlideStep']
                self.SlideBlockSize = para_dict['SlideBlockSize']
                slide_dis = self.SlideStep * self.SlideBlockSize
                if slide_dis <= 0 or slide_dis >= self.BufferDis:
                    self.init_succeed = False
                    self.failed_inf.append(
                        'slide_dis <= 0 or slide_dis >= BufferDis (slide_dis = SlideStep * SlideBlockSize)')

                self.MaxIteration = para_dict['MaxIteration']
                self.TargetMeanLoss = para_dict['TargetMeanLoss']
                self.IterStep = para_dict['IterStep']
                self.UpperLimitOfGaussNewteon = para_dict['UpperLimitOfGaussNewteon'] * self.MaxIteration
                if self.MaxIteration <= 1 or self.TargetMeanLoss < 1 or self.IterStep <= 0:
                    self.init_succeed = False
                    self.failed_inf.append('MaxIteration <= 1 or TargetMeanLoss < 1 or IterStep <= 0')

                if self.EmdFilterLevel <= 0:
                    self.init_succeed = False
                    self.failed_inf.append('EmdFilterLevel <= 0')
                temp = para_dict['TransfersProduceConfig']
                if len(temp) == 2 and len(temp[0]) == 3 and len(temp[1]) == 3:
                    temp[0][2] = math.radians(temp[0][2])
                    self.TransfersProduceConfig = temp
                else:
                    self.init_succeed = False
                    self.failed_inf.append('TransfersProduceConfig wrong size.')
                # TODO 从MagMapFiles中读出首行的各种信息，检查是否有问题
                self.MagMapFiles = para_dict['MagMapFiles']
                self.CoordinateOffset = para_dict['CoordinateOffset']
                self.EntranceList = para_dict['EntranceList']
                if len(self.CoordinateOffset) == 0 or len(self.EntranceList) == 0:
                    self.init_succeed = False
                    self.failed_inf.append('CoordinateOffset or EntranceList wrong size.')

                # 本系统作为socket服务器的端口
                self.ServerPort = para_dict['ServerPort']
                if self.ServerPort < 1 or self.ServerPort > 65535:
                    self.init_succeed = False
                    self.failed_inf.append('ServerPort < 1 or ServerPort > 65535')

                # 与本地的其它程序通信所使用的本地socket
                # self.LocalPortMagposition = para_dict['LocalPortMagposition']
                # self.LocalPortPdr = para_dict['LocalPortPdr']

                # self.LocalSocketMagposition = socket.socket()
                # if self.LocalPortMagposition < 1 or self.LocalPortMagposition > 65535:
                #     self.init_succeed = False
                #     self.failed_inf.append('LocalPortMagposition < 1 or LocalPortMagposition > 65535')
                # else:
                #     self.LocalSocketMagposition.connect(('127.0.0.1', self.LocalPortMagposition))

                # self.LocalSocketPdr = socket.socket()
                # if self.LocalPortPdr < 1 or self.LocalPortPdr > 65535:
                #     self.init_succeed = False
                #     self.failed_inf.append('LocalPortPdr < 1 or LocalPortPdr > 65535')
                # else:
                #     self.LocalSocketPdr.connect(('127.0.0.1', self.LocalPortPdr))

        except ConnectionRefusedError as ce:
            self.failed_inf.append(['对方程序未开启socket服务', ce])
            self.init_succeed = False
        except Exception as e:
            self.failed_inf.append(['其它异常', e])
            self.init_succeed = False

        # 如果初始化失败，断开可能已经建立的socket连接
        # if self.init_succeed is False:
        #     if self.LocalSocketMagposition is not None:
        #         self.LocalSocketMagposition.close()
        #     if self.LocalSocketPdr is not None:
        #         self.LocalSocketPdr.close()

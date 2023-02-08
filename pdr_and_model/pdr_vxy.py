import quaternion
import numpy as np
import torch
import pdr_and_model.model_resnet1d as rm
import matplotlib.pyplot as plt

_input_channel, _output_channel = 6, 2
_fc_config = {'fc_dim': 512, 'in_dim': 7, 'dropout': 0.5, 'trans_planes': 128}


def workpart(IMU, model):
    if not torch.cuda.is_available():
        device = torch.device('cpu')
        checkpoint = torch.load(model, map_location=lambda storage, location: storage)
    else:
        device = torch.device('cuda:0')
        checkpoint = torch.load(model)
    IMU = np.array(IMU)
    acce = IMU[:, 0:3]
    gyro = IMU[:, 3:6]
    ori = IMU[:, 6:10]

    # 这句话不是很确定是否要把w提到第一列
    ori = ori[:, [3, 0, 1, 2]]

    ori_q = quaternion.from_float_array(ori)
    gyro_q = quaternion.from_float_array(np.concatenate([np.zeros([gyro.shape[0], 1]), gyro], axis=1))
    acce_q = quaternion.from_float_array(np.concatenate([np.zeros([acce.shape[0], 1]), acce], axis=1))
    glob_gyro = quaternion.as_float_array(ori_q * gyro_q * ori_q.conj())[:, 1:]
    glob_acce = quaternion.as_float_array(ori_q * acce_q * ori_q.conj())[:, 1:]

    network = rm.ResNet1D(_input_channel, _output_channel, rm.BasicBlock1D, [2, 2, 2, 2],
                          base_plane=64, output_block=rm.FCOutputModule, kernel_size=3, **_fc_config)
    # network = torch.nn.DataParallel(network)
    network.load_state_dict(checkpoint['model_state_dict'])
    network.eval().to(device)
    IMU = np.concatenate([glob_gyro, glob_acce], axis=1)
    IMU = torch.unsqueeze(torch.Tensor(IMU.T), 0)
    x_y = network(IMU.to(device)).cpu().detach().numpy()

    return x_y


if __name__ == '__main__':
    IMU = [[0 for _ in range(10)] for _ in range(200)]
    # filename = r"C:\Users\HJH\Desktop\IMU-88-7-270.6518297687728 Pixel 6_sync.csv"
    # v = []
    # data = np.loadtxt(filename, delimiter=',')
    # for i in range(0,len(data)-200,200):
    #     IMU = data[i:i+200, 1:11]
    #     v.append(workpart(IMU,model)[0])
    # v = np.array(v)
    # x = np.zeros([v.shape[0]+2, 2])
    # x[0] =[0,0]
    # x[1:-1] = np.cumsum(v[:, :2], axis=0)
    # x[-1] = x[-2]
    #
    # plt.plot(x[:, 0], x[:, 1])
    # plt.show()
    model = 'ronin.pt'
    print(workpart(IMU, model))

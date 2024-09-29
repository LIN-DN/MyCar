import numpy as np

## 参数类
class SenorConfig:
    def __init__(self):
        ## imu参数
        self.eul_n2b_bias = np.deg2rad([0,0,0])               # 初始时刻IMU坐标系相对全局坐标系的偏转角

        ## uwb参数
        self.UWB_COM = '/dev/ttyACM0'
        self.UWB_BAUD = 115200
        self.UWB_freq = 50                                  # Hz
        self.MAX_NUM_ANCS = 8
        self.UES_NUM_ANCS = 4
        self.tag_id = 1    # 标签序号
        self.tag_z = 0.28  # 假设标签高度固定


        self.anchor_position =  np.array([[0.0, 0.6, 0.52],
                                          [1.2, 3.6, 0.52],
                                          [4.8, 3.0, 0.52],
                                          [3.6, 0.0, 0.52]])

        self.standard_anchor = np.array([[0.0, 0.0, 0.23],
                                         [0.0, 4.8, 0.23],
                                         [4.8, 4.8, 0.23],
                                         [4.8, 0.0, 0.23],
                                         [0.0, 2.4, 1.765],
                                         [2.4, 4.8, 1.765],
                                         [4.8, 2.4, 1.765],
                                         [1.8, 0.0, 1.765]])     # 标定基站位 根据实际情况修改（标签尽可能在基站所围区域内）
        self.standard_tag = np.array([[1.2, 1.2, 0.31],
                                      [1.2, 3.6, 0.31], 
                                      [2.4, 2.4, 0.31],
                                      [3.6, 1.2, 0.31],
                                      [3.6, 3.6, 0.31]])     # 标定测试位 根据实际情况修改（尽可能在多个位点测试,且对于同一基站真实距离因有差异）
        self.recording_time = 5                              # 标定时间 每个测试位记录测试数据的时长 根据需要修改
        # UWB 标定系数 根据标定结果修改
        # self.a = [0.9902, 0.9875, 0.9807, 0.984, 0.9974, 0.9865, 0.983, 0.9801]      # V2.0
        # self.b = [-396.1, -314.3, -314.92, -363.71, -361.9, -301.31, -388.15, -304.74]
        self.a = [0.9903, 0.9844, 0.9906, 0.984,  0.9835, 0.9851, 0.9838, 0.9703]      # V1.0 T1 同平面标定
        self.b = [-564.85, -535.66, -584.47, -533.81, -528.41, -502.36, -560.92, -383.46]      

        ## 融合器参数
        self.fusion_dim = 3            # 2: EKF 融合采用2D模式,  3: EKF融合采用3D模式
        self.m_div_cntr = 0            # 量测分频器
        self.m_div = 2                 # 每m_div次量测，才更新一次EKF量测(UWB更新),  可以节约计算量 或者 做实验看效果

        ## 滤波器参数
        # Initial Kalman filter noise estimation
        self.sigma_uwb = 0.1                        # UWB测距噪声
        self.sigma_acc = 0.03                       # 加速度计噪声
        self.sigma_gyro = np.deg2rad(0.3)           # 陀螺仪噪声
        self.sigma_acc_bias = 0.05                  # 加速度计零偏随机游走噪声
        self.sigma_gyro_bias = np.deg2rad(0.05)     # 陀螺仪零偏随机游走噪声
        # Initial Kalman filter uncertainties (standard deviations)
        self.factp = np.zeros(7)                    # 初始状态方差
        self.factp[0] = 1                           # 初值位置方差(置信度)(m)，越大代表对初始位置越不信任
        self.factp[1] = 1                           # 初值速度方差([m / s]
        self.factp[2:5] = np.deg2rad([1, 1, 10])    # 初始i姿态方差(roll, pitch, yaw)[rad]
        self.factp[5] = 0.01                        # 初始加速度零偏方差[m / s ^ 2]
        self.factp[6] = np.deg2rad(0.01)            # 初始角速度零偏方差[rad / s]

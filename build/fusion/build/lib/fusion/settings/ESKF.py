from . import utils
from . import config 
import numpy as np
from scipy.linalg import block_diag
from transforms3d.euler import quat2euler, euler2quat

class ESKF:
    """
        # UWB IMU 融合算法，采用误差卡尔曼15维经典模型，伪距组合
        # UWB：                 PR(TOF) 伪距
        # IMU:                 加速度(3) 3轴陀螺(3) 共6维,,
        # normal_state:        名义状态: 导航方程状态: 位置(3) 速度(3) 四元数(4) 共10维
        # err_state:           KF误差状态: 位置误差(3) 速度误差(3) 失准角(3) 加速度计零偏误差(3) 陀螺零偏误差(3) 共15维5
        # du:                  零偏反馈: 加速度计零偏(3) 陀螺零偏(3)， 共6维

        # 单位说明:
        # 加速度,加速度零偏: m/s^(2)
        # 角速度, 角速度(陀螺)零偏: rad/s
        # 角度 rad
        # 速度: m/s
    """

    def __init__(self):
        self.settings = config.SenorConfig()
        self.normal_state = np.concatenate((np.zeros((6, 1)), np.reshape(euler2quat(0, 0, 0), (-1, 1))))
        self.err_state = np.zeros((15, 1))
        self.du = np.zeros((6, 1))

        # self.anchor = self.settings.anchor_position.T
        # self.anchor_num = self.settings.anchor_position.shape[0]          # 基站个数 N
        self.dim = self.settings.fusion_dim

        self.init_filter()

    ## 初始化滤波器
    def init_filter(self):
        # Kalman filter state matrix
        self.P = np.zeros((15, 15))
        self.P[0:3, 0:3] = self.settings.factp[0] ** 2 * np.eye(3)
        self.P[3:6, 3:6] = self.settings.factp[1] ** 2 * np.eye(3)
        self.P[6:9, 6:9] = np.diag(self.settings.factp[2:5]) ** 2
        self.P[9:12, 9:12] = self.settings.factp[5] ** 2 * np.eye(3)
        self.P[12:15, 12:15] = self.settings.factp[6] ** 2 * np.eye(3)

        # Process noise covariance
        self.Q1 = np.zeros((6, 6))
        self.Q1[0:3, 0:3] = self.settings.sigma_acc ** 2 * np.eye(3)
        self.Q1[3:6, 3:6] = self.settings.sigma_gyro ** 2 * np.eye(3)

        self.Q2 = np.zeros((6, 6))
        self.Q2[0:3, 0:3] = self.settings.sigma_acc_bias ** 2 * np.eye(3)
        self.Q2[3:6, 3:6] = self.settings.sigma_gyro_bias ** 2 * np.eye(3)

    ## 状态空间模型
    def state_space_model(self, x, acc, dt):
        Cb2n = utils.ch_q2m(x[6:10])
        sk = utils.ch_askew(Cb2n @ acc)
        # Approximation of the discrete time state transition matrix
        # F = [[O, I, O, O, O],
        #      [O, O, - sk, - Cb2n, O],
        #      [O, O, O, O, - Cb2n],
        #      [O, O, O, O, O],
        #      [O, O, O, O, O]]

        F = np.zeros((15, 15))
        F[0:3, 3:6] = np.eye(3)
        F[3:6, 6:9] = -sk
        F[3:6, 9:12] = -Cb2n
        F[6:9, 12:15] = -Cb2n
        F = np.eye(15) + dt * F

        # Noise gain matrix
        # G = [[O, O, O, O],
        #     [Cb2n, O, O, O],
        #     [O, - Cb2n, O, O],
        #     [O, O, I, O],
        #     [O, O, O, I]]

        G = np.zeros((15, 12))
        G[3:6, 0:3] = Cb2n
        G[6:9, 3:6] = -Cb2n
        G[9:12, 6:9] = np.eye(3)
        G[12:15, 9:12] = np.eye(3)
        G = dt * G

        return F, G

    ## Y 估计伪距 H 观测矩阵(雅可比矩阵)
    def uwb_hx(self, x, anchor):
        position = x[:3]
        position[2] = self.settings.tag_z
        # anchor = self.anchor
        N = anchor.shape[1]  # 基站个数

        if self.dim == 2:
            position = position[:2]
            # anchor = self.anchor[:2, :N]
            anchor = anchor[:2, :N]

        Y = []
        H = np.zeros((0, 15))
        # 计算预测的伪距s
        perd_pr = np.tile(position, (1, N)) - anchor[:, :N]
        for i in range(N):
            # 伪距归一化
            normalized_pr = perd_pr[:, i] / np.linalg.norm(perd_pr[:, i])
            if self.dim == 2:
                H = np.vstack((H, np.hstack((normalized_pr, np.zeros(13)))))
            else:
                H = np.vstack((H, np.hstack((normalized_pr, np.zeros(12)))))
            Y.append(np.linalg.norm(perd_pr[:, i]))
        return Y, H

    ## IMU惯导解算更新(当地直角坐标系，不考虑地球自转)
    def imu_INS(self, p, v, q, acc, gyr, dt):
        """
        :param p: 位置 XYZ 单位 m                    3x1
        :param v: 速度 XYZ 单位 m/s                  3x1
        :param q: Qb2n姿态, 四元数表示                4x1
        :param acc: 比力, 加速度计测量值 单位(m/s^2)    3x1
        :param gyr: 角速度(rad/s)                    3x1
        :param dt: 积分间隔(s) 如 0.01s
        :return: IMU 解算位姿 position velocity quaternion
        """
        old_v = v
        sf = acc.copy()
        gN = np.reshape([0, 0, - 9.8], (-1, 1))  # 东北天坐标系，重力简化为-9.8
        # 姿态解算 也可以直接用解算好的欧拉角转换成四元数
        # q = utils.ch_att_upt(q, gyr, dt)  # 4 x 1

        # 速度解算
        sf = utils.ch_qmulv(q, sf)  # 加速度b2n
        # print(f"本体加速度{acc.reshape(1,-1)} 全局加速度{sf.reshape(1,-1)}")
        sf = sf + gN  # 在n系下消除重力影响
        v = old_v + dt * sf  # 积分更新速度           # 3 x 1

        # 位置解算
        p = p + (old_v + v) * dt / 2  # 3 x 1

        return p, v, q


    # UWB最小二乘法多边测距
    def uwb_MLAT(self, pos, pr, anchor):
        """
        M x N: M:维度 2 OR 3  N: 基站个数
        :param pos: 定位u坐标 M x 1
        :param pr:  量测伪距 N x 1
        :param dim: 2 or 3: 2: 二维定位 3: 三维定位
        :return: 更新坐标
        """
        B1 = 0.1
        END_LOOP = 100
        max_retry = 5
        lpos = pos.copy()  # Save the previous position
        anchor_num = anchor.shape[1]

        # if self.anchor_num < 3:
        if anchor_num < 3:
            return pos

        while END_LOOP > B1 and max_retry > 0:
            # Calculate the current distance from the user to each base station
            # r = np.linalg.norm(self.anchor - pos, axis=0)  # 伪距 1 x N
            r = np.linalg.norm(anchor - pos, axis=0)
            # Calculate the H matrix (Jacobian Matrix)
            # H = (self.anchor - pos) / r  # 雅可比矩阵 M x N
            H = (anchor - pos) / r
            if self.dim == 2:
                H = np.hstack((H, np.array([[0], [0], [-1]])))
            H = -H.T
            dp = (pr - r.reshape(-1, 1))  # 伪距残差 N x 1
            if self.dim == 2:
                dp = np.vstack((dp, 0))

            # Iteratively update the user's position
            delta = (np.linalg.inv(H.T @ H) @ H.T @ dp)  # 位置更新步长 M x 1

            # Calculate the residual
            END_LOOP = np.linalg.norm(delta)

            # Update the position
            pos = pos + delta
            max_retry = max_retry - 1

            # If iteration fails
            if max_retry == 0 and END_LOOP > 10:
                pos = lpos
                return pos

        return pos


    def predict(self, acc, gyr, quat, dt):
        # 反馈 加速度bias, 陀螺bias
        # acc = acc - self.du[0:3]
        # gyr = gyr - self.du[3:6]

        # 捷联惯导解算
        p = self.normal_state[0:3]
        v = self.normal_state[3:6]
        # q = self.normal_state[6:10]
        q = quat
        # print(f"前{self.normal_state[:6].reshape(1,-1)}")
        p, v, q = self.imu_INS(p, v, q, acc, gyr, dt)

        # 小车假设：基本做平面运动，N系下Z轴速度基本为0，直接给0; 高度已知，Z轴坐标约束
        v[2] = 0
        p[2] = self.settings.tag_z

        self.normal_state[0:3] = p
        self.normal_state[3:6] = v
        self.normal_state[6:10] = q
        # print(f"后{self.normal_state[:6].reshape(1,-1)}")

        # 生成F阵 15x15   G阵 15x12
        F, G = self.state_space_model(self.normal_state, acc, dt)

        # 卡尔曼P阵预测公式 15x15
        self.P = F @ self.P @ F.T + G @ block_diag(self.Q1, self.Q2) @ G.T

    def correct(self):
        # 车载约束：Z轴速度约束： B系下 Z轴速度必须为0(不能钻地).. 可以有效防止Z轴位置跳动
        R2 = np.eye(1) * 0.5
        Cn2b = utils.ch_q2m(utils.ch_qconj(self.normal_state[6:10]))

        H = np.concatenate((np.zeros(3), np.dot(np.array([0, 0, 1]), Cn2b), np.zeros(9)))  # 1 x 15
        
        K = ((self.P @ H.T) / (H @ self.P @ H.T + R2)).reshape(-1, 1)  # 15 x 1  #原方程
        z = Cn2b @ self.normal_state[3:6]  # 将速度投影到b系
        self.err_state = np.concatenate((np.zeros((9, 1)), self.du)) + (K * (0 - z[2]))

        # 反馈速度位置
        self.normal_state[0:6] = self.normal_state[0:6] + self.err_state[0:6]

        # # 反馈姿态
        # q = self.normal_state[6:10]
        # q = utils.ch_qmul(utils.ch_rv2q(self.err_state[6:9]), q)
        # self.normal_state[6:10] = q

        # 存储加速度计零偏，陀螺零偏
        self.du = self.err_state[9:15]

        # P阵后验更新
        self.P = (np.eye(15) - K * H) @ self.P

    def update(self, pr, anchor):
        anchor_num = anchor.shape[1] # N
        R1 = np.eye(anchor_num) * (self.settings.sigma_uwb ** 2)  # N x N

        # 量测方程
        Y, H = self.uwb_hx(self.normal_state, anchor)  # Y 1xN H Nx15

        # 卡尔曼公式，计算K
        S = H @ self.P @ H.T + R1  # system uncertainty N x N
        residual = pr - Y  # 实际测量距离 - 估计伪距 = residual 或者叫新息 1 x N

        # Self-Calibrating Multi-Sensor Fusion with Probabilistic
        # 如果量测置信度比较大，则更不相信量测
        L = residual @ np.linalg.inv(S) @ residual.T
        if L > 20:
            S = H @ self.P @ H.T + R1 * 5   # 根据量测置信度给R一些增益

        K = (self.P @ H.T) @ np.linalg.inv(S)

        self.err_state = np.concatenate((np.zeros((9, 1)), self.du)) + (K @ residual.T).reshape(-1, 1)

        # 反馈速度位置
        # print(f"前{self.normal_state[:6].reshape(1,-1)}")
        self.normal_state[:6] = self.normal_state[:6] + self.err_state[:6]
        # print(f"后{self.normal_state[:6].reshape(1,-1)}")
        # 反馈姿态
        # q = self.normal_state[6:10]
        # q = utils.ch_qmul(utils.ch_rv2q(self.err_state[6:9]), q)
        # self.normal_state[6:10] = q

        # 存储加速度计零偏，陀螺零偏
        self.du = self.err_state[9:15]

        # P阵后验更新
        self.P = (np.eye(15) - K @ H) @ self.P

    def getState(self):
        pos = np.round(self.normal_state[0:3],3).reshape(1,-1)[0]
        vel = np.round(self.normal_state[3:6],4).reshape(1,-1)[0]
        quat = self.normal_state[6:10].reshape(1,-1)[0]
        return pos, vel, quat

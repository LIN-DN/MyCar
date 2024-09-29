import numpy as np
from numpy import reshape, round, rad2deg, deg2rad
from transforms3d.euler import quat2euler, euler2quat


## 共轭四元数
def ch_qconj(qin):
    qout = np.array([qin[0], -qin[1], -qin[2], -qin[3]])
    return qout

## 速度解算
def ch_qmulv(q, vi):
    # Inputs: q - Qb2n  vi - 需要旋转的向量
    # Output: vout - output vector, such that vout = q * vin * conjugation(q)
    qo1 = -q[1][0] * vi[0][0] - q[2][0] * vi[1][0] - q[3][0] * vi[2][0]
    qo2 = q[0][0] * vi[0][0] + q[2][0] * vi[2][0] - q[3][0] * vi[1][0]
    qo3 = q[0][0] * vi[1][0] + q[3][0] * vi[0][0] - q[1][0] * vi[2][0]
    qo4 = q[0][0] * vi[2][0] + q[1][0] * vi[1][0] - q[2][0] * vi[0][0]
    vo = vi
    vo[0] = -qo1 * q[1][0] + qo2 * q[0][0] - qo3 * q[3][0] + qo4 * q[2][0]
    vo[1] = -qo1 * q[2][0] + qo3 * q[0][0] - qo4 * q[1][0] + qo2 * q[3][0]
    vo[2] = -qo1 * q[3][0] + qo4 * q[0][0] - qo2 * q[2][0] + qo3 * q[1][0]
    return vo

## 四元数相乘
def ch_qmul(q1, q2):
    # Inputs: Q1 Q2, 四元数和矩阵一样，不满足交换律
    # Outputs: Q
    q = np.zeros((4,1))
    q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
    q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return q




eul1 = deg2rad([0,0,1])
eul2 = deg2rad([0,0,91])
quat1 = euler2quat(eul1[0],eul1[1],eul1[2])
quat2 = euler2quat(eul2[0],eul2[1],eul2[2])
# quat_bias = ch_qmul(quat1,quat2)
quat_bias = ch_qmul(quat1,ch_qconj(quat2))
eul_bias = rad2deg(quat2euler(quat_bias))
print(quat_bias, eul_bias)
acc_b = np.array([1,0,0])
acc_n = ch_qmulv(quat2.reshape(-1,1),acc_b.reshape(-1,1).copy())
print(f"本体加速度{acc}, 偏转欧拉角{eul1}, 四元数{quat1},全局加速度{acc_n.reshape(1,-1)}") 
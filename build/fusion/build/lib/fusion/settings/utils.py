import numpy as np
import math


## 生成反对称矩阵
def ch_askew(v):
    # Input: v - 3 x1 vector
    # Output: m - v的反对称阵
    m = np.array([[0, -v[2][0], v[1][0]],
                  [v[2][0], 0, -v[0][0]],
                  [-v[1][0], v[0][0], 0]])
    return m

## 将欧拉角转换为姿态阵
def ch_eul2m(att, model='312'):
    # Input: att 单位：rad
    # 对于312((Z->X->Y))顺序，对应att = [pitch(绕X轴) roll(绕Y轴)  yaw(绕Z轴)]
    # 对于3211(Z->Y->X)顺序，对应att = [roll(绕X轴) pitch(绕Y轴)  yaw(绕Z轴)]
    # Outputs:
    # Cb2n_312: 312欧拉角顺序下转换后的Cb2n
    # Cb2n_321: 321欧拉角顺序下转换后的Cb2n

    Cb2n = []
    s = np.sin(att)
    c = np.cos(att)
    si = s[0]
    sj = s[1]
    sk = s[2]
    ci = c[0]
    cj = c[1]
    ck = c[2]

    if model == '312':
        Cb2n = [[cj * ck - si * sj * sk, -ci * sk, sj * ck + si * cj * sk],
                [cj * sk + si * sj * ck, ci * ck, sj * sk - si * cj * ck],
                [-ci * sj, si, ci * cj]]
    elif model == '321':
        Cb2n = [[cj * ck, si * sj * ck - ci * sk, ci * sj * ck + si * sk],
                [cj * sk, si * sj * sk + ci * ck, ci * sj * sk - si * ck],
                [-sj, si * cj, ci * cj]]

    return Cb2n

## 四元数转姿态阵
def ch_q2m(Qb2n):
    # Input: Qb2n
    # Output: Cb2n
    q0 = Qb2n[0][0]
    q1 = Qb2n[1][0]
    q2 = Qb2n[2][0]
    q3 = Qb2n[3][0]

    q11 = q0 * q0
    q12 = q0 * q1
    q13 = q0 * q2
    q14 = q0 * q3
    q22 = q1 * q1
    q23 = q1 * q2
    q24 = q1 * q3
    q33 = q2 * q2
    q34 = q2 * q3
    q44 = q3 * q3

    Cb2n = np.array([[q11 + q22 - q33 - q44, 2 * (q23 - q14), 2 * (q24 + q13)],
                     [2 * (q23 + q14), q11 - q22 + q33 - q44, 2 * (q34 - q12)],
                     [2 * (q24 - q13), 2 * (q34 + q12), q11 - q22 - q33 + q44]])

    return Cb2n

## 姿态阵转四元数
def ch_m2q(Cb2n):
    # Input: Cb2n
    # Output: Qb2n

    C11 = Cb2n[0][0]
    C12 = Cb2n[0][1]
    C13 = Cb2n[0][2]
    C21 = Cb2n[1][0]
    C22 = Cb2n[1][1]
    C23 = Cb2n[1][2]
    C31 = Cb2n[2][0]
    C32 = Cb2n[2][1]
    C33 = Cb2n[2][2]
    if C11 >= C22 + C33:
        q1 = 0.5 * np.sqrt(1 + C11 - C22 - C33)
        q0 = (C32 - C23) / (4 * q1)
        q2 = (C12 + C21) / (4 * q1)
        q3 = (C13 + C31) / (4 * q1)
    elif C22 >= C11 + C33:
        q2 = 0.5 * np.sqrt(1 - C11 + C22 - C33)
        q0 = (C13 - C31) / (4 * q2)
        q1 = (C12 + C21) / (4 * q2)
        q3 = (C23 + C32) / (4 * q2)
    elif C33 >= C11 + C22:
        q3 = 0.5 * np.sqrt(1 - C11 - C22 + C33)
        q0 = (C21 - C12) / (4 * q3)
        q1 = (C13 + C31) / (4 * q3)
        q2 = (C23 + C32) / (4 * q3)
    else:
        q0 = 0.5 * np.sqrt(1 + C11 + C22 + C33)
        q1 = (C32 - C23) / (4 * q0)
        q2 = (C13 - C31) / (4 * q0)
        q3 = (C21 - C12) / (4 * q0)

    Qb2n = [q0, q1, q2, q3]
    return Qb2n

## 欧拉角转四元数
def ch_eul2q(eul, model='312'):
    Cb2n = ch_eul2m(eul, model)
    Qb2n = ch_m2q(Cb2n)
    return Qb2n

## 四元数转欧拉角
def ch_q2eul(Qb2n, model='312'):
    # Input: 四元数Qb2n
    # Outputs:
    # eul 312(Z->X->Y)旋转顺序下的欧拉角: att = [pitch(绕X轴) roll(绕Y轴) yaw(绕Z轴)]
    # eul 321(Z->Y->X)旋转顺序下的欧拉角: att = [roll(绕X轴) pitch(绕Y轴)  yaw(绕Z轴)]
    q0 = Qb2n[0][0]
    q1 = Qb2n[1][0]
    q2 = Qb2n[2][0]
    q3 = Qb2n[3][0]
    eul = []
    # print("旋转顺序",model)
    if model == '312':
        roll = -np.arctan2(2 * (q1 * q3 - q0 * q2), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)
        pitch = np.arcsin(2 * (q0 * q1 + q2 * q3))
        yaw = -np.arctan2(2 * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3)
        eul = [pitch, roll, yaw]
    elif model == '321':
        roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * q1 * q1 - 2 * q2 * q2)
        pitch = np.arcsin(2 * (q0 * q2 - q1 * q3))
        yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * q2 * q2 - 2 * q3 * q3)
        eul = [roll, pitch, yaw]
    return eul

## 等效旋转矢量转换为变换四元数
def ch_rv2q(rv):
    nm2 = np.dot(rv.T, rv)  # 旋转矢量的模方
    if nm2 < 1.0e-8:  # 如果模方很小，则可用泰勒展开前几项求三角函数
        q0 = 1 - nm2 * (1 / 8 - nm2 / 384)
        s = 1 / 2 - nm2 * (1 / 48 - nm2 / 3840)
    else:
        nm = np.sqrt(nm2)
        q0 = np.cos(nm / 2)
        s = np.sin(nm / 2) / nm
    q = np.vstack([q0, s * rv])
    return q

## 四元数相乘
def ch_qmul(q1, q2):
    # Inputs: Q1 Q2, 四元数和矩阵一样，不满足交换律 左乘 先做q2变换再做q1变换
    # Outputs: Q
    q = np.zeros((4,1))
    q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
    q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return q

## 四元数归一化
def ch_qnormlz(q):
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return q

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

## 姿态解算
def ch_att_upt(in_q, gyr, dt):
    # 单子样旋转矢量
    rv = gyr * dt  # 旋转角 rad
    dq = ch_rv2q(rv)

    out_q = ch_qmul(in_q, dq)
    out_q = ch_qnormlz(out_q)

    return out_q
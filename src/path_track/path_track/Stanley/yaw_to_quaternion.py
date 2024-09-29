from math import cos, sin, atan2

from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(heading):
    ''' 
    Converts yaw heading to quaternion coordinates.
    将偏航航向转换为四元数坐标。 输入弧度
    '''
    theta = 0.5 * heading
    quaternion = Quaternion()
    quaternion.z = sin(theta)
    quaternion.w = cos(theta)

    return quaternion

def quaternion_to_theta(quaternion):
    """
    将四元数坐标转换为偏航。
    """
    # 计算四元数的角度部分
    theta = 2 * atan2(quaternion.z, quaternion.w)
    # print(f"theta ={theta}")
    # # 将弧度转换为角度
    # yaw = theta * (180 / 3.14159)
    return theta



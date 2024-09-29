from .settings import utils
from .settings import config
from .settings import ESKF
import time
import threading
import numpy as np
from numpy import reshape, round, rad2deg, deg2rad
from transforms3d.euler import quat2euler, euler2quat
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
np.set_printoptions(suppress=True)


class FUSION(Node):

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        # 常量参数
        self.settings = config.SenorConfig()
        self.eskf = ESKF.ESKF()

        self.isINIT = False
        self.hasPreTime = False

        self.previous_timestamp = 0.0
        self.anchor_list = self.settings.anchor_position

        self.Q_zero_bias = []
        self.Pose_list = []
        self.Quat_list = []

        # 订阅话题 imu uwb传感器数据
        self.imu_sub = self.create_subscription(Imu,"/imu",self.imuCallback,10)
        self.uwb_sub = self.create_subscription(Float64MultiArray,'uwb_distance',self.uwbCallback,1)

        # 发布话题 融合坐标
        self.fused_pose_pub = self.create_publisher(PoseStamped, 'Fused/pose', 1)    # 话题发布机制

    # ESKF初始化
    def state_initialize(self, uwb_msg):
        pr_list = np.array(uwb_msg.data)
        valid_pr_index = np.nonzero(pr_list)[0] # 有效性检验
        if len(valid_pr_index) < 3:
            self.get_logger().info("当前UWB有效基站少于3个,请检查电量!")
            return
        anchor_pr = pr_list[valid_pr_index]
        anchor_position = self.anchor_list[valid_pr_index]
        pos = self.eskf.uwb_MLAT(reshape([1, 0, 0.0], (-1, 1)), anchor_pr.reshape(-1,1), anchor_position.T)
        pos[2] = self.settings.tag_z
        self.eskf.normal_state[0:3] = pos
        self.isINIT = True

    # 根据imu消息进行状态预测
    def imuCallback(self, imu_msg):
        if self.isINIT:
            # imu 四元数
            quat_b = np.array([imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z]).reshape(-1,1)

            # 去掉静偏
            quat_mid = utils.ch_qmul(utils.ch_qconj(self.Q_zero_bias), quat_b)
            
            # 坐标系转化
            eul_n2b_bias = self.settings.eul_n2b_bias
            quat_n2b_bias = euler2quat(eul_n2b_bias[0],eul_n2b_bias[1],eul_n2b_bias[2])
            quat_n = utils.ch_qmul(quat_n2b_bias, quat_mid)

            acc = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z]).reshape(-1,1)
            gyr = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z]).reshape(-1,1)
            # print(f"加速度{acc.reshape(1,-1)}")
            current_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
            dt = round(current_timestamp - self.previous_timestamp, 4)
            self.eskf.predict(acc, gyr, quat_n, dt)
            self.eskf.correct()
            self.previous_timestamp = current_timestamp

        else:
            self.previous_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
            self.Q_zero_bias = np.array([imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z])
            # print(f"IMU初始姿态累积偏差{round(rad2deg(quat2euler(self.Q_zero_bias)),2)}")
            self.hasPreTime = True
 

    # 根据uwb消息进行状态更新
    def uwbCallback(self, uwb_msg):
        if self.hasPreTime:
            if self.isINIT :
                pr_list = np.array(uwb_msg.data)
                valid_pr_index = np.nonzero(pr_list)[0] # 有效性检验
                if len(valid_pr_index) < self.settings.UES_NUM_ANCS:
                    self.get_logger().info(f"当前UWB有效基站少于{self.settings.UES_NUM_ANCS}个,请检查电量!")
                    return
                anchor_pr = pr_list[valid_pr_index]
                anchor_position = self.anchor_list[valid_pr_index]
                self.eskf.update(anchor_pr, anchor_position.T)                
                self.eskf.correct()
                self.fusedPub()
            else:
                self.state_initialize(uwb_msg)  # 使用uwb进行初始定位
                pos, vel, quat = self.eskf.getState()
                eul = round(rad2deg(quat2euler(quat)),2)
                self.get_logger().info(f'初始坐标{pos}(m) 速度{vel}(m/s) 姿态{eul}(°)')

    def fusedPub(self):
        now = self.get_clock().now()

        pos, vel, quat = self.eskf.getState()
        eul = round(rad2deg(quat2euler(quat)),3)
        self.get_logger().info(f'坐标{pos}(m) 速度{vel}(m/s) 姿态{eul}(°)')

        Fused_pose = PoseStamped()
        Fused_pose.header.stamp = now.to_msg()
        Fused_pose.header.frame_id = 'map'

        Fused_pose.pose.position.x = pos[0]
        Fused_pose.pose.position.y = pos[1]
        # Fused_pose.pose.position.z = pos[2]
        Fused_pose.pose.position.z = 0.0

        Fused_pose.pose.orientation.w = quat[0]
        Fused_pose.pose.orientation.x = quat[1]
        Fused_pose.pose.orientation.y = quat[2]
        Fused_pose.pose.orientation.z = quat[3]
        self.fused_pose_pub.publish(Fused_pose)

def main(args=None):
    rclpy.init(args=args)             # 初始化rclpy
    node = FUSION('Fusion_node')      # 新建一个节点
    rclpy.spin(node)                  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown()                  # 关闭rclpy

from ..settings import config
import numpy as np
import time
import serial
from scipy.optimize import least_squares
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node



# UWB话题类
class uwbLocation(Node):

    # UWB初始化
    def __init__(self,name):
        # ROS话题发布
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.uwb_sub = self.create_subscription(Float64MultiArray,'uwb_distance',self.uwbCallback,1)
        self.uL_path_pub = self.create_publisher(Path, 'uwbLocation/path', 1)

        # 初始化基站坐标
        settings = config.SenorConfig()
        self.use_anchors = [0, 1, 2, 3]
        self.ancs_position = settings.anchor_position[self.use_anchors]
        self.position = [0, 0, 0]
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.Pose_list = []

    # 计算距离残差
    def distance_residuals(self, tag_position, anchors, distances):
        # 计算预测距离
        tag_position[2] = 0.28
        predicted_distances = np.linalg.norm(anchors - tag_position, axis=1)
        # 计算残差
        residuals = predicted_distances - distances
        return residuals
              
    def uwbCallback(self, uwb_msg):
        now = self.get_clock().now()
        distance = np.array(uwb_msg.data)[self.use_anchors]
        valid_pr_index = np.nonzero(distance)[0] # 有效性检验
        if len(valid_pr_index) < len(self.use_anchors):
            self.get_logger().info(f"当前UWB有效基站少于{len(self.use_anchors)}个,请检查电量!")
            return
        # 最小二乘法计算标签坐标
        self.position[2] = 0.29
        result = least_squares(self.distance_residuals, self.position, args=(self.ancs_position, distance))
        # self.position = np.round(result.x, 3)
        self.position = result.x
        print(f"纯UWB定位: {self.position} 测距{distance}")

        self.Pose_list.append(self.position)

        uL_path = Path()
        uL_path.header.stamp = now.to_msg()
        uL_path.header.frame_id = 'map'

        for i in range(len(self.Pose_list)):
            msg = PoseStamped()

            msg.pose.position.x = self.Pose_list[i][0]
            msg.pose.position.y = self.Pose_list[i][1]
            msg.pose.position.z = self.Pose_list[i][2]

            msg.pose.orientation.w = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0

            uL_path.poses.append(msg)

        self.uL_path_pub.publish(uL_path)
                    

def main(args=None):
    rclpy.init(args=args)        # 初始化rclpy
    node = uwbLocation('UWB_location')    # 新建一个节点
    rclpy.spin(node)            # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()            # 关闭rclpy
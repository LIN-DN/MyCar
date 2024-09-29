import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class fusedPath(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.Pose_list = []
        self.Quat_list = []

        self.fused_sub = self.create_subscription(PoseStamped,'Fused/pose',self.fusedCallback,1)
        self.fused_path_pub = self.create_publisher(Path, 'Fused/path', 1)

    def fusedCallback(self, pose_msg):
        
        now = self.get_clock().now()
        pos = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
        quat = [pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z]

        self.Pose_list.append(pos)
        self.Quat_list.append(quat)

        Fused_path = Path()
        Fused_path.header.stamp = now.to_msg()
        Fused_path.header.frame_id = 'map'

        for i in range(len(self.Pose_list)):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'Fused_path'

            msg.pose.position.x = self.Pose_list[i][0]
            msg.pose.position.y = self.Pose_list[i][1]
            msg.pose.position.z = self.Pose_list[i][2]

            msg.pose.orientation.w = self.Quat_list[i][0]
            msg.pose.orientation.x = self.Quat_list[i][1]
            msg.pose.orientation.y = self.Quat_list[i][2]
            msg.pose.orientation.z = self.Quat_list[i][3]

            Fused_path.poses.append(msg)

        self.fused_path_pub.publish(Fused_path)


def main(args=None):
    rclpy.init(args=args)                   # 初始化rclpy
    node = fusedPath('fusion_path_publisher')   # 新建一个节点 
    rclpy.spin(node)                        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()                        # 关闭rclpy

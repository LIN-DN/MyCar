import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

np.set_printoptions(suppress=True)

class ERROR(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.Mocap_pose = []
        self.Fused_pose = []
        self.Error_list = []


        self.Mocap_sub = self.create_subscription(PoseStamped,"/car1_Marker1/pose",self.MocapCallback,1)
        self.Fused_sub = self.create_subscription(PoseStamped,'Fused/pose',self.FusedCallback,1)
        self.Error_pub = self.create_publisher(Float64, 'Error', 1)    # 话题发布机制

    def MocapCallback(self, msg):
        self.Mocap_pose = np.array([round(msg.pose.position.x,3), round(msg.pose.position.y,3)])


    def FusedCallback(self, msg):
        self.Fused_pose = np.array([round(msg.pose.position.x,3), round(msg.pose.position.y,3)])   
        if len(self.Mocap_pose) != 0 and self.Fused_pose.all():
            Error = Float64()
            Error.data = np.round(np.linalg.norm(self.Mocap_pose - self.Fused_pose , axis=0), 3)    

            print(f"动捕坐标{self.Mocap_pose} 融合坐标{self.Fused_pose} 融合误差{Error.data}")
            self.Error_pub.publish(Error)


def main(args=None):
    rclpy.init(args=args)                   # 初始化rclpy
    node = ERROR('error_analyse')           # 新建一个节点
    rclpy.spin(node)                        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()                        # 关闭rclpy

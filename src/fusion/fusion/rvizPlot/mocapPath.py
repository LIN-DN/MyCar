import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from transforms3d.euler import quat2euler
np.set_printoptions(suppress=True)

class Mocap(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.Pose_list = []
        self.Quat_list = []

        self.Mocap_sub = self.create_subscription(PoseStamped,"/car1_Marker1/pose",self.MocapCallback,1)
        self.Mocap_path_pub = self.create_publisher(Path, 'Mocap/path', 1)

    def MocapCallback(self, pose_msg):
        
        now = self.get_clock().now()
        pos = [round(pose_msg.pose.position.x,3), round(pose_msg.pose.position.y,3), round(pose_msg.pose.position.z,3)]
        quat = [pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z]
        eul = np.round(np.rad2deg(quat2euler(quat)),2)
        print(f"动捕坐标{pos}")

        self.Pose_list.append(pos)
        self.Quat_list.append(quat)

        Mocap_path = Path()
        Mocap_path.header.stamp = now.to_msg()
        Mocap_path.header.frame_id = 'map'

        for i in range(len(self.Pose_list)):
            msg = PoseStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'Mocap_path'

            msg.pose.position.x = self.Pose_list[i][0]
            msg.pose.position.y = self.Pose_list[i][1]
            msg.pose.position.z = 0.29
            msg.pose.orientation.w = self.Quat_list[i][0]
            msg.pose.orientation.x = self.Quat_list[i][1]
            msg.pose.orientation.y = self.Quat_list[i][2]
            msg.pose.orientation.z = self.Quat_list[i][3]

            Mocap_path.poses.append(msg)

        self.Mocap_path_pub.publish(Mocap_path)

def main(args=None):
    rclpy.init(args=args)                   # 初始化rclpy
    node = Mocap('motion_capture_subscriber')      # 新建一个节点
    rclpy.spin(node)                        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown()                        # 关闭rclpy

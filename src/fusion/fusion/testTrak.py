import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

np.set_printoptions(suppress=True)

class PosPub(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.Pos_List = [[0.0, 0.0], [0.1, 0.0], [0.2, 0.0], [0.3, 0.0], [0.4, 0.0], [0.4, 0.1], [0.4, 0.2], [0.4, 0.3], [0.4, 0.4], [0.5, 0.5]]
        self.Pos_Pub = self.create_publisher(PoseStamped, 'UAV_pos', 1)    # 话题发布机制
        self.Pos_id = 0

        timer_period = 2  # 每1s解算一次
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def timer_callback(self):
        print(f"目标点{self.Pos_List[self.Pos_id]}")
        pos_msg = PoseStamped()
        if self.Pos_id == 10:
            self.Pos_id = 0
        pos_msg.pose.position.x = self.Pos_List[self.Pos_id][0]
        pos_msg.pose.position.y = self.Pos_List[self.Pos_id][1]

        self.Pos_Pub.publish(pos_msg)

        self.Pos_id += 1
        print(self.Pos_id)

def main(args=None):
    rclpy.init(args=args)                   # 初始化rclpy
    node = PosPub('traking_pos_pub')        # 新建一个节点
    rclpy.spin(node)                        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()                        # 关闭rclpy

if __name__=='__main__':
    main()
import time
import threading
import re
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from socket import *
import struct

np.set_printoptions(suppress=True)

class Traking(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.uav_x_bias = 1.6
        self.uav_y_bias = 2.7
        self.uav_pre_timestamp = 0.0
        self.uav_timestamp = 0.0

        self.flag = False

        self.Car_pos = [0.0, 0.0]
        self.UAV_pos = [0.0, 0.0]

        self.trakeFreq = 1
        self.trakeVel = [0.0, 0.0]

        # # 作为服务器
        # # 设置TCP服务器的地址和端口
        # HOST = "10.51.128.167"
        # POST = 12345

        # # 创建TCP服务器并监听端口
        # server_socket = socket(AF_INET, SOCK_STREAM)
        # server_socket.bind((HOST, POST))
        # server_socket.listen(1)
        # self.client_socket, address = server_socket.accept()
        # print("Connected to:", address)

        # self.test_sub = self.create_subscription(PoseStamped, 'UAV_pos', self.Test,1 )
        self.Fused_sub = self.create_subscription(PoseStamped, 'Fused/pose', self.Cmd_Pub,1 )
        self.Vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)    # 话题发布机制

        # 作为客户端
        server_host = "10.51.129.67"
        server_port = 45678

        self.client_socket = socket(AF_INET, SOCK_STREAM)
        self.client_socket.connect((server_host, server_port))

        self.UAV_Client = threading.Thread(target=self.TCP_Update)
        self.UAV_Client.start()


    def TCP_Update(self):
        while True:
            # 接收串口数据
            data= self.client_socket.recv(1024).decode()
            self.uav_timestamp = time.time()
            # print(data)

            # 解码数据
            pos = data.split(",")
            # print(pos)
            self.UAV_pos[0] = round(float(pos[0]) + self.uav_x_bias, 3)
            self.UAV_pos[1] = round(float(pos[1]) + self.uav_y_bias, 3)

            if float(pos[2][:8]) > 0.5:
                # print(f"UAV已升高至{float(pos[2][:8])}")
                self.flag = True

            # print(f"追踪目标:{self.UAV_pos}")
        # # 关闭TCP连接
        # server_socket.close()

    def Test(self, msg):
        self.uav_timestamp = time.time()
        self.UAV_pos[0] = round(msg.pose.position.x + self.uav_x_bias, 3)
        self.UAV_pos[1] = round(msg.pose.position.y + self.uav_y_bias, 3)
        # print(f"追踪目标{self.UAV_pos}")

    def Cmd_Pub(self, msg):
        self.Car_pos = np.array([round(msg.pose.position.x + 0.15,3), round(msg.pose.position.y,3)])
        # print(f"追踪目标{self.UAV_pos}, 当前坐标{self.Car_pos}")
        if self.uav_timestamp != self.uav_pre_timestamp and self.flag:
            self.uav_pre_timestamp = self.uav_timestamp
            distance = np.linalg.norm(self.UAV_pos - self.Car_pos , axis=0)
            # print(f"追踪目标{self.UAV_pos}, 当前坐标{self.Car_pos}, 目标距离{distance}")
            if distance > 0.05:
                x_vel = round((self.UAV_pos[0] - self.Car_pos[0]) / self.trakeFreq, 2)
                y_vel = round((self.UAV_pos[1] - self.Car_pos[1]) / self.trakeFreq, 2)
                print(f"追踪目标{self.UAV_pos}, 当前坐标{self.Car_pos}, 目标距离{distance}, cmd_vel:{x_vel, -y_vel}")
                if abs(x_vel) <= 1.1 and abs(y_vel) <= 1.1:
                    cmd_msg = Twist()
                    cmd_msg.linear.x = x_vel
                    cmd_msg.linear.y = -y_vel
                    self.Vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)                   # 初始化rclpy
    node = Traking('Traking_UAV')           # 新建一个节点
    rclpy.spin(node)                        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()                        # 关闭rclpy

if __name__=='__main__':
    main()
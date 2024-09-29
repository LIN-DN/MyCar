from .settings import config
import numpy as np
import time
import serial
import struct
from scipy.optimize import least_squares
from std_msgs.msg import Header, Float64MultiArray
import rclpy
from rclpy.node import Node

def statistics(mask):
	Acitive_Anchors = []
	for i in range(8):
		if mask & 1 == 1:
			Acitive_Anchors.append(i)
		mask = mask >> 1
	return Acitive_Anchors

# UWB话题类
class uwbNode(Node):

    # UWB初始化
    def __init__(self,name):
        # ROS话题发布
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")
        self.distance_pub = self.create_publisher(Float64MultiArray, 'uwb_distance', 1)

        # UWB常量参数
        setting = config.SenorConfig()
        self.COM = setting.UWB_COM
        self.BAUD = setting.UWB_BAUD
        self.MAX_NUM_ANCS = setting.MAX_NUM_ANCS
        
        # 初始化标签序号和基站坐标
        self.tag_id = setting.tag_id
        self.ancs_position = setting.anchor_position

        # 基站标定系数
        self.a = setting.a
        self.b = setting.b

        # 标签距离数据
        self.measured_pr = [0 for i in range(self.MAX_NUM_ANCS)]  # 原始uwb距离
        self.distance = [0 for i in range(self.MAX_NUM_ANCS)]  # 回归标定距离

        try:
            self.uwb_ser = serial.Serial(port=self.COM, baudrate=self.BAUD)
            if self.uwb_ser.isOpen():
                print("\033[32mUWB串口打开成功...\033[0m")
            else:
                self.uwb_ser.open()
                print("\033[32m打开UWB串口成功...\033[0m")
        except Exception as e:
            print(e)
            print("\033[31mUWB串口打开失败\033[0m")
            exit(0)
        else:  
            self.monitor_uwb()  


    # 解析UWB数据
    def parse_UWBdata(self, buf):
        if len(buf) == 101 and buf[:6] == b"CmdM:4" and buf[99:101] == b"\r\n":
            # 标签序号
            index = 11
            tagid = struct.unpack("<H", buf[index:index + 2])[0]
            # print(f"tagid {tagid}")

            # 基站序号
            index += 2
            anchorid = struct.unpack("<H", buf[index:index + 2])[0]
            # print(f"anchorid {anchorid}")

            # 序列号
            index += 2
            seq = buf[index]
            # print(f"seq {seq}")

            # mask校验位置
            index += 1
            mask = buf[index]
            Acitive_Anchors = statistics(mask)
            
            if len(Acitive_Anchors) == 0:
                print(f"当前无可用基站，请检查电量！")
                return False
            else:
                print(f"当前{Acitive_Anchors}号基站可用")

            # 原始距离数据
            index = 17
            rawrange = []
            for i in range(self.MAX_NUM_ANCS):
                rawrange.append(struct.unpack("<I", buf[index:index + 4])[0])
                index += 4

            # 卡尔曼滤波结果
            index += 1
            kalmanrange = []
            for i in range(self.MAX_NUM_ANCS):
                kalmanrange.append(struct.unpack("<I", buf[index:index + 4])[0])
                index += 4
            # print(kalmanrange)
        else:
            # print("接受数据错误")
            return False

        return tagid, rawrange, kalmanrange, Acitive_Anchors

    # 实时读取UWB数据
    def monitor_uwb(self):
        while True:
            data_count = self.uwb_ser.inWaiting()
            if data_count > 0:
                data_buff = self.uwb_ser.read(data_count)
                data = self.parse_UWBdata(data_buff)
                if not data:
                    continue
                if data[0] == self.tag_id:
                    for i, value in enumerate(data[2]):
                        if i not in data[3]:
                            self.measured_pr[i] = 0.0
                        elif value != 0:
                            self.measured_pr[i] = round((self.a[i] * value + self.b[i])/1000,3)
                    # self.distance = [round((a_i * x_i + b_i) / 1000, 3) for x_i, a_i, b_i in zip(self.measured_pr, self.a, self.b)]
                    self.distance = self.measured_pr
                    # 发布UWB距离话题
                    distance_msg = Float64MultiArray()
                    distance_msg.data = self.distance
                    self.distance_pub.publish(distance_msg)
                    print(f'UWB测距{distance_msg.data}')
                    # self.get_logger().info(f'UWB测距{distance_msg.data}')

                
def main(args=None):
    rclpy.init(args=args)
    node = uwbNode('UWB_publisher')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
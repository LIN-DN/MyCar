import sys
sys.path.insert(0,sys.path[0] + "/..")  # 将当前路径变为上级目录

from settings import config
import numpy as np
import time
import serial
import struct
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

def statistics(mask):
	count = 0
	for i in range(8):
		if mask & 1 == 1:
			count += 1
		mask = mask >> 1
	return count

class RECORD(Node):
    # UWB初始化
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")
        # UWB常量参数
        settings = config.SenorConfig()
        self.MAX_NUM_ANCS = settings.MAX_NUM_ANCS
        self.recording_time = settings.recording_time
        self.COM = settings.UWB_COM
        self.BAUD = settings.UWB_BAUD
        self.a = []
        self.b = []

        # 初始化基站坐标
        self.ancs_positions = settings.standard_anchor
        self.USE_ANCS = self.ancs_positions.shape[0]  # 基站个数

        # 标签距离数据
        self.pr = [0 for i in range(self.USE_ANCS)]        # 原始uwb距离
        self.real_dis_list = []                            # 真实标-基距离
        self.measure_dis_list = []                         # 测量平均标-基距离
        self.record_list = []                              # 数据记录列表

        self.tag_id = settings.tag_id
        self.tag_real_coord = settings.standard_tag
        self.isFinished = False
        self.R1 = False
        self.B = False

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

        # 遥控记录
        print(f"请点击 R1 开始标定")
        self.control_sub = self.create_subscription(Joy,'joy',self.controlCallback,1)
        self.record_num = 0 

    # 自动化记录
    def controlCallback(self, control_msg):  
            # 开启新位置标定 R1
            if control_msg.buttons[5] == 1:
                if not self.isFinished:
                    self.R1 = True
                    print(f"请将标签置于坐标点{self.tag_real_coord[self.record_num]}处并点击 L1 开始记录")
            # 开始当前位置记录 L1
            if self.R1:
                if control_msg.buttons[4] == 1:
                    self.recording()
            # 删除最新录制记录 B
            if control_msg.buttons[1] == 1 and self.B:
                self.RollbackRecord()
            # 记录完毕开始解算 A
            if self.isFinished:
                if control_msg.buttons[0] == 1:
                    self.Calculation_coefficient()
    def recording(self):
        # if self.record_num == self.tag_real_coord.shape[0] :
        #     print(f"* * * * * 所有测试位均已记录，如需撤销本次记录请点击按键 B ，或点击按键 A 开始计算标定系数 * * * * *\n")
        #     return False
        print(f"\n* * * * * * * * * * 开始第{self.record_num+1}处记录* * * * * * * * * * ")
        real_pos = self.tag_real_coord[self.record_num]
        real_dis = np.round(np.linalg.norm(real_pos - self.ancs_positions, axis=1),3)
        self.real_dis_list.append(real_dis)
        print(f"> > > 当前位置与基站真实距离为{self.real_dis_list[self.record_num]} < < <")

        th_uwb = threading.Thread(target=self.monitor_uwb)     
        th_uwb.start()
        th_uwb.join() #timeout=self.recording_time

        if not th_uwb.is_alive():
            measure_dis = np.mean(self.record_list, axis=0)
            self.measure_dis_list.append(measure_dis)
            print(f"> > > 当前位置与基站测量距离为{self.measure_dis_list[self.record_num]} < < <")

            self.record_list = []
            self.R1 = False
            self.B = True
            self.record_num += 1
            if not self.record_num == self.tag_real_coord.shape[0] :
                print(f"* * * * * 第{self.record_num}处测距记录结束，如需撤销本次记录请点击按键 B ，或点击 R1 开启下一位置标定 * * * * *\n")
            else:
                print(f"* * * * * 所有测试位均已记录，如需撤销本次记录请点击按键 B ，或点击按键 A 开始计算标定系数 * * * * *\n")
                self.isFinished = True

    def RollbackRecord(self):
        if self.record_num == 1:
            self.real_dis_list = []
            self.measure_dis_list = []
            self.record_num = 0
        else:
            self.real_dis_list.pop()
            self.measure_dis_list.pop()
            self.record_num -= 1
        
        print(f"! ! ! 第{self.record_num+1}处记录已删除，请点击 R1 重新记录 ! ! !")
        self.B = False
        self.isFinished = False
        self.R1 = False

    # 计算系数
    def Calculation_coefficient(self):
        print(f"真实距离{self.real_dis_list}")
        print(f"测量距离{self.measure_dis_list}")
        for k in range(self.USE_ANCS):
            x_k = np.array(self.measure_dis_list)[:,k]*1000
            y_k = np.array(self.real_dis_list)[:,k]*1000
            a_k, b_k = np.polyfit(x_k, y_k, 1)
            self.a.append(a_k)
            self.b.append(b_k)
        print(f"该标签标定系数A为{np.round(self.a, 4)},请更新config.SenorConfig.self.a")
        print(f"该标签标定系数B为{np.round(self.b, 2)},请更新config.SenorConfig.self.b")
        self.isFinished = False

    # 解析UWB数据
    def parse_UWBdata(self, buf):
        if len(buf) == 101 and buf[:6] == b"CmdM:4" and buf[99:101] == b"\r\n":
            self.uwb_timestamp = round(time.time(), 4)

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
            count = statistics(mask)
            # print(f"当前{count}个基站可用")
            if count != self.USE_ANCS:
                # print(f"校验位不匹配 当前{count}个基站可用")
                return False

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

        return tagid, rawrange, kalmanrange

    # 实时读取UWB数据
    def monitor_uwb(self):

        print_num = 0
        start_time = time.time()
        while time.time() - start_time < 10:
            data_count = self.uwb_ser.inWaiting()
            if data_count > 0:
                data_buff = self.uwb_ser.read(data_count)
                # print(data_buff)
                data = self.parse_UWBdata(data_buff)
                if not data:
                    continue
                if data[0] == self.tag_id:
                    for i, value in enumerate(data[2]):
                        if value != 0:
                            self.pr[i] = value / 1000
                    # distance
                    self.record_list.append(self.pr)
                    if print_num % 50 == 0:
                        print(f"UWB测距数据{self.pr}")
                    print_num += 1

if __name__ == "__main__":
    rclpy.init()                   # 初始化rclpy
    node = RECORD('standard_subscriber')      # 新建一个节点
    rclpy.spin(node)                        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()                        # 关闭rclpy

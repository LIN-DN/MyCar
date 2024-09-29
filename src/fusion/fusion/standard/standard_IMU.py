import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
np.set_printoptions(suppress=True)

class IMU(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.acc = []
        self.gyr = []
        self.eul = []
        
        self.acc_pre = []
        self.gyr_pre = []
        self.eul_pre = []

        self.acc_0 = []
        self.gyr_0 = []
        self.eul_0 = []

        self.acc_bias_list = []
        self.gyr_bias_list = []
        self.eul_bias_list = []


        self.recording_time = 300 # s
        self.time_flag = False
        self.unEnd = True
        self.fist_flag = True
        self.start_time = time.time()
        print(f"* * * * * IMU零偏矫正程序启动,记录时间{self.recording_time/60}min * * * * *")

        self.imu_sub = self.create_subscription(Imu,"/imu",self.imuCallback,1)
        self.timer = self.create_timer(1, self.timer_callback)  #启动一个定时装置，每 1 s,调用一次time_callback函数

    # 记录IMU静止状态下的偏移量
    def imuCallback(self, imu_msg):
        if time.time() - self.start_time < self.recording_time:
            self.acc = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])
            self.gyr = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
            quat = [imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z]
            self.eul = np.rad2deg(quat2euler(quat))

        else:
            self.time_flag = True
            if self.unEnd:
                self.calculate()  
        if self.fist_flag:
            self.acc_pre = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])
            self.gyr_pre = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
            quat = [imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z]
            self.eul_pre = np.rad2deg(quat2euler(quat))

            self.acc_0 = self.acc_pre.copy()
            self.gyr_0 = self.gyr_pre.copy()
            self.eul_0 = self.eul_pre.copy()
            print(f"首帧状态{self.acc_0,self.gyr_0,self.eul_0}")

            self.fist_flag = False

    def timer_callback(self):
        if not self.time_flag:
            # print(f"pre姿态{self.eul_pre}, now姿态{self.eul}")
            acc_bias = np.array(self.acc) - np.array(self.acc_pre)
            gyr_bias = np.array(self.gyr) - np.array(self.gyr_pre)
            eul_bias = np.array(self.eul) - np.array(self.eul_pre)
            self.acc_bias_list.append(acc_bias)
            self.gyr_bias_list.append(gyr_bias)
            self.eul_bias_list.append(eul_bias)
            self.acc_pre = self.acc.copy()
            self.gyr_pre = self.gyr.copy()
            self.eul_pre = self.eul.copy()

            countdown = int(time.time() - self.start_time)
            if countdown % 60 == 0:
                print(f"记录倒计时{(self.recording_time - countdown)/60}min")
                print(f"当前状态{self.acc,self.gyr,self.eul}")

    def calculate(self):
        self.unEnd = False

        print(f"首帧状态{self.acc_0,self.gyr_0,self.eul_0} \n末帧状态{self.acc,self.gyr,self.eul} \n记录次数{len(self.acc_bias_list)}")

        ax_bias = np.mean(np.array(self.acc_bias_list)[:,0])
        ay_bias = np.mean(np.array(self.acc_bias_list)[:,1])
        az_bias = np.mean(np.array(self.acc_bias_list)[:,2])
        gx_bias = np.mean(np.array(self.gyr_bias_list)[:,0])
        gy_bias = np.mean(np.array(self.gyr_bias_list)[:,1])
        gz_bias = np.mean(np.array(self.gyr_bias_list)[:,2])
        ex_bias = np.mean(np.array(self.eul_bias_list)[:,0])
        ey_bias = np.mean(np.array(self.eul_bias_list)[:,1])
        ez_bias = np.mean(np.array(self.eul_bias_list)[:,2])
        print(f"加速度计偏移{np.round(np.array([ax_bias, ay_bias, az_bias]),4)},  \
                陀螺仪偏移{np.round(np.array([gx_bias, gy_bias, gz_bias]),4)},    \
                姿态偏移{np.round(np.array([ex_bias, ey_bias, ez_bias]),4)}(deg/s)")

if __name__ == "__main__":
    rclpy.init()                   # 初始化rclpy
    node = IMU('IMU_subscriber')      # 新建一个节点
    rclpy.spin(node)                        # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()                        # 关闭rclpy
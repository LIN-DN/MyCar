#!/usr/bin/env python3
'''
    MPC轨迹追踪器
'''
import threading
import numpy as np
import math 
import time
from .MPC.control import Car_Dynamics, MPC_Controller, Linear_MPC_Controller

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker

class Tracker(Node):

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        # parameter
        self.receding = 8                       # MPC Horizon窗口长度
        self.control_frequency = 1/30           # 车辆状态采样时间 5Hz
        self.max_speed = 0.10                    # 最大车速
        self.max_steer = 0.5                    # 最大转向角
        self.shape = [0.28, 0.20, 0.18, 0.16]   # [length, width, wheelbase, wheelbase_w] limo

        # 初始化类
        self.my_car = Car_Dynamics(0, 0, 0, np.deg2rad(0), length=self.shape[0], dt=self.control_frequency)
        self.controller = MPC_Controller()

        self.path = []
        self.target_id = 0
        self.pre_target_id = 0
        self.min_distance = 0

        # 线程锁
        self.lock = threading.Lock()

        # 回调组
        # self.pose_cbG = MutuallyExclusiveCallbackGroup()
        # self.vel_cbG = MutuallyExclusiveCallbackGroup()
        # self.path_cbG = MutuallyExclusiveCallbackGroup()

        # ros topic
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        # self.car_pose_pub = self.create_publisher(PoseStamped, '/rearstate_point', 1)
        # self.ref_point_pub = self.create_publisher(Marker, '/ref_point', 1)
        self.ref_point_pub = self.create_publisher(PoseStamped, '/ref_point', 1)

        self.pose_sub = self.create_subscription(PoseStamped, 'Fused/pose', self.vehicle_pose_cb, 1)#, callback_group=self.pose_cbG
        # self.pose_sub = self.create_subscription(PoseStamped, "/tracked_pose", self.vehicle_pose_cb, 1, callback_group=self.state_cb)
        self.vel_sub = self.create_subscription(Odometry, "/odom", self.vehicle_vel_cb, 1)#, callback_group=self.vel_cbG
        # self.path_sub = self.create_subscription(Path, '/Trajectory', self.path_cb, 1, callback_group=self.path_cb)
        self.path_sub = self.create_subscription(Path, '/record_path', self.path_cb, 10)#, callback_group=self.path_cbG

        # self.track_timer = self.create_timer(self.control_frequency, self.timer_cb)

    def timer_cb(self):
        if len(self.path) > 0:
            self.MPC_control()

    def publish_car_pose(self,x:float,y:float,orientation):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = orientation

        self.car_pose_pub.publish(pose_msg)

    def publish_ref_point(self):
        # marker = Marker()
        # marker.header.frame_id = 'map'
        # marker.type = Marker.SPHERE
        # marker.action = Marker.ADD
        # marker.pose.position.x = self.path[self.target_id][0]
        # marker.pose.position.y = self.path[self.target_id][1]
        # marker.pose.position.z = 0.22
        # # marker.pose.orientation = Tracker.yaw_to_quat(self.path[self.target_id][2])

        # # 大小
        # marker.scale.x = 0.1
        # marker.scale.y = 0.1
        # marker.scale.z = 0.1
        # # 颜色
        # marker.color.r = 1.0
        # marker.color.a = 1.0
        # self.ref_point_pub.publish(marker)

        target_msg = PoseStamped()
        target_msg.header.frame_id = "map"
        target_msg.pose.position.x = self.path[self.target_id][0]
        target_msg.pose.position.y = self.path[self.target_id][1]
        target_msg.pose.position.z = 0.0
        target_msg.pose.orientation = Tracker.yaw_to_quat(self.path[self.target_id][2])

        self.ref_point_pub.publish(target_msg)

    def vehicle_pose_cb(self, msg):
        """
        更新小车坐标定位
        """
        self.lock.acquire()

        x = round(msg.pose.position.x,3)
        y = round(msg.pose.position.y,3)
        quat = msg.pose.orientation  
        raw = Tracker.quat_to_yaw(quat)

        # offset = 0.1
        
        # self.my_car.x = x - offset * np.cos(raw)
        # self.my_car.y = y - offset * np.sin(raw)
        self.my_car.x = x
        self.my_car.y = y
        self.my_car.psi = raw
        # print(f"小车当前位置{x, y} 偏航角{np.rad2deg(raw)}")

        # self.publish_car_pose(self.my_car.x, self.my_car.y, quat)

        # if len(self.path) > 0:
        #     self.min_distance, self.target_id = Tracker.closest_point([self.my_car.x, self.my_car.y],self.path, self.target_id)
        #     self.publish_ref_point()
        if len(self.path) > 0:
            # time_start = time.time()
            self.MPC_control()
            # print(time.time()-time_start)
        self.lock.release()

    def vehicle_vel_cb(self, msg):
        """
        更新小车速度状态
        """
        # self.lock.acquire()

        self.my_car.v = msg.twist.twist.linear.x
        # print(f"小车速度{self.my_car.v}")

        # self.lock.release()

    def path_cb(self, msg):
        """
        订阅追踪轨迹
        """
        self.lock.acquire()

        self.set_vehicle_command(0.0, 0.0)

        # 只执行当前轨迹
        self.path = []
        self.target_id = 0
        self.pre_target_id = 0
        self.min_distance = 0
        
        for i in range(0, len(msg.poses)):
            point = msg.poses[i].pose
            px = point.position.x
            py = point.position.y
            praw = Tracker.quat_to_yaw(point.orientation)
            self.path.append([px, py, praw])
        self.path = np.array(self.path)
        print("收到新的轨迹数据")

        self.lock.release()

    def MPC_control(self):
        # self.lock.acquire()

        # print("* * * * * * * * * *")

        if len(self.path) > 0:
            self.min_distance, self.target_id = Tracker.closest_point([self.my_car.x, self.my_car.y, self.my_car.psi, self.my_car.v], self.path, self.target_id)
            self.publish_ref_point()

        vector_ve2tg = [self.path[self.target_id][0]-self.my_car.x, self.path[self.target_id][1]-self.my_car.y]
        vector_vehicle = [self.my_car.v * np.cos(self.my_car.psi), self.my_car.v * np.sin(self.my_car.psi)]
        angle_ve2tg = np.arccos(np.dot(vector_ve2tg, vector_vehicle) / (np.linalg.norm(vector_ve2tg) * np.linalg.norm(vector_vehicle)))
        # print(f"追踪方向{np.rad2deg(angle_ve2tg)}")

        # print(f"car state: 位置{self.my_car.x, self.my_car.y} 偏航角{np.rad2deg(self.my_car.psi)}")
        # print(f"target: id {self.target_id} 坐标{np.round(self.path[self.target_id][0:2],3)} 偏转角{np.rad2deg(self.path[self.target_id][2])}")
        
        acc, delta = self.controller.optimize(self.my_car, self.path[self.target_id:self.target_id+self.receding])

        # print(f"MPC: 加速度{acc} 待转角{np.rad2deg(delta)} 速度{self.my_car.v} ") #速度{self.my_car.v} 
        self.my_car.update_state(self.my_car.move(acc,  delta))

        vel = self.my_car.v
        steer = delta

        # 下发控制
        if self.target_id == len(self.path)-1:
            print("到达终点!!!")
            self.path = []
            vel = 0.0
            steer = 0.0
        else:
            vel = min(self.max_speed, max(-self.max_speed, vel))
            steer = min(self.max_steer, max(-self.max_steer, steer))

        # 新版小车设计特殊性
        if vel < 0.0:
            steer = -steer

        if angle_ve2tg > np.deg2rad(90) and angle_ve2tg < -np.deg2rad(90):
            vel = 0.0
            steer = 0.0

        # self.get_logger().info(f'actions: linear{round(vel, 3)} steer{np.rad2deg(steer)}')
        # print(f'actions: linear{round(vel, 3)} steer{np.rad2deg(steer)}')
        self.set_vehicle_command(vel, steer)

        # self.lock.release()

    def set_vehicle_command(self, velocity, steering_angle):
        ''' Publishes the calculated steering angle  '''
        drive = Twist()
        drive.linear.x = velocity
        drive.linear.y = 0.0
        drive.linear.z = 0.0

        drive.angular.x = 0.0
        drive.angular.y = 0.0
        drive.angular.z = steering_angle

        self.cmd_pub.publish(drive)

    @staticmethod
    def quat_to_yaw(quater):
         
        w = quater.w
        x = quater.x
        y = quater.y
        z = quater.z

        raw = np.arctan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    @staticmethod
    def yaw_to_quat(yaw):
         
        w = np.cos(yaw/2)
        x = 0.0
        y = 0.0
        z = np.sin(yaw/2)

        quat = Quaternion(w=w, x=x, y=y, z=z)

        return quat
    
    @staticmethod
    def closest_point(car_state, points, start_ind, threshold=0.01, ind_range=10):
        
        min_dis = np.inf
        min_ind = start_ind
    
        for i, waypoint in enumerate(points[start_ind:start_ind+ind_range]):

            # 确保追踪点保持在行进方向“前方“
            vector_ve2tg = [waypoint[0]-car_state[0], waypoint[1]-car_state[1]]
            vector_vehicle = [car_state[3] * np.cos(car_state[2]), car_state[3] * np.sin(car_state[2])]
            angle_ve2tg = np.arccos(np.dot(vector_ve2tg, vector_vehicle) / (np.linalg.norm(vector_ve2tg) * np.linalg.norm(vector_vehicle)))
            if angle_ve2tg < np.deg2rad(60) and angle_ve2tg > -np.deg2rad(60):
                dis = math.sqrt((car_state[0] - waypoint[0])**2 + (car_state[1] - waypoint[1])**2) 
                if dis < min_dis:
                    min_dis = dis
                    min_ind = start_ind + i

                    # if dis < threshold:
                    #     break

        return min_dis, min_ind
    
    @staticmethod
    def wraptopi(radian):
        while radian > np.pi:
            radian = radian - 2 * np.pi
        while radian < -np.pi:
            radian = radian + 2 * np.pi

        return radian

def main(args=None):
    rclpy.init(args=args)             # 初始化rclpy
    node = Tracker('MPC_Trake_node')  # 新建一个节点
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.spin()                  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.spin(node)                  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown()                  # 关闭rclpy

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
'''
    Stanley轨迹追踪器
'''
from .Stanley import normalise_angle
import threading
import numpy as np
from numpy import reshape, round, rad2deg, deg2rad
from transforms3d.euler import quat2euler, euler2quat

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist,PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from nav_msgs.msg import Path

class Tracker(Node):

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")


        # Initialise subscribers 
        self.localisation_sub = self.create_subscription(PoseStamped, '/car1/pose', self.vehicle_state_cb, 10)
        # self.localisation_sub = self.create_subscription(PoseStamped, 'Fused/pose', self.vehicle_state_cb, 10)
        # self.path_sub = self.create_subscription(Path, '/Trajectory', self.path_cb, 10)
        self.path_sub = self.create_subscription(Path, '/record_path', self.path_cb, 10)

        # Initialise publishers
        self.tracker_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 横向参考点
        self.Traking_point_pub = self.create_publisher(PointStamped, '/Traking_point', 10)
        self.Frontaxle_point_pub = self.create_publisher(PointStamped, '/Frontaxle_point', 10)

        # 车辆体积
        self.car_pub = self.create_publisher(Marker, 'car', 10)
        # 车辆姿态
        # self.car_pose_pub = self.create_publisher(PoseStamped, 'car_pose_pub', 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', 50.0),
                    ('control_gain', 10),
                    ('softening_gain', 1.0),
                    ('yawrate_gain', 1.0),
                    ('steering_limits', 0.98),
                    ('centreofgravity_to_frontaxle', 0.28)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.k = float(self.get_parameter("control_gain").value)
            self.ksoft = float(self.get_parameter("softening_gain").value)
            self.kyaw = float(self.get_parameter("yawrate_gain").value)
            self.max_steer = float(self.get_parameter("steering_limits").value)
            # 重心到前轴
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except ValueError:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.yaw = None
        self.target_vel = 0.0
        self.yaw_flag = True
        self.stop = True

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.target_idx = 0
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        # 线程锁
        self.lock = threading.Lock()
        self.dt = 1 / self.frequency

        # Intialise timers
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def publish_rectangle(self,x:float,y:float,orientation):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'my_namespace'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = orientation[0]
        marker.pose.orientation.x = orientation[1]
        marker.pose.orientation.y = orientation[2]
        marker.pose.orientation.z = orientation[3]
        # 大小
        marker.scale.x = 0.35
        marker.scale.y = 0.25
        marker.scale.z = 0.2
        # 颜色
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.car_pub.publish(marker)

    def publish_car_pose(self,x:float,y:float,orientation):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = orientation

        self.car_pose_pub.publish(pose_msg)

    def timer_cb(self):
        # if not self.stop:
        # 控制
        self.stanley_control()

    def vehicle_state_cb(self, msg):
        """
        更新车辆状态
        """
        self.lock.acquire()

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = quat2euler([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])[2]
        # print(f"小车当前位置{self.x, self.y, rad2deg(self.yaw)}")
        # 发布小车体积
        # self.publish_rectangle(self.x, self.y, euler2quat(0.0, 0.0, self.yaw))
        # 发布小车位置
        # self.publish_car_pose(self.x, self.y, euler2quat(0.0, 0.0, self.yaw))

        # 只要有路径，且没有走到路径终点，就不停（保持追踪）
        if self.cyaw and self.target_idx != len(self.cyaw)-1:
            self.stop = False
            self.target_index_calculator()
        # 如果已经走到终点，就停止
        elif self.target_idx == len(self.cyaw)-1:
            print("轨迹追踪结束")
            self.stop = True
            self.target_idx = 0
            self.cx = []
            self.cy = []
            self.cyaw = []
            self.target_vel = 0.0
            self.set_vehicle_command(0.0, 0.0)

        self.lock.release()

    def path_cb(self, msg):
        """
        订阅追踪轨迹
        """
        self.lock.acquire()

        # 只执行当前轨迹
        self.cx = []
        self.cy = []
        self.cyaw = []
        for i in range(0, len(msg.poses)):
            point = msg.poses[i].pose
            px = point.position.x
            py = point.position.y
            pyaw = quat2euler([point.orientation.w, point.orientation.x, point.orientation.y, point.orientation.z])[2]
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(pyaw)
        print(f"收到新的轨迹数据{len(self.cx)}")
        self.lock.release()

    def target_vel_cb(self, msg):
        """
        设置车辆速度
        """
        self.target_vel = msg.data

    def target_index_calculator(self):

        '''
            Calculates the target index and each corresponding error
            计算目标索引和相应的误差
        '''
        # 计算前轴中心点位置  
        fx = self.x + self.cg2frontaxle * np.cos(self.yaw)
        fy = self.y + self.cg2frontaxle * np.sin(self.yaw) 

        # 绘制小车前轴中心点
        frontaxle_point = PointStamped()
        frontaxle_point.header.frame_id = "map"
        frontaxle_point.header.stamp = self.get_clock().now().to_msg()
        frontaxle_point.point.x = fx
        frontaxle_point.point.y = fy
        self.Frontaxle_point_pub.publish(frontaxle_point)

        # 求前轴相对于路径的x、y偏差
        dx = [fx - icx for icx in self.cx]
        dy = [fy - icy for icy in self.cy]
        # 求前轴到路径的距离
        d = np.hypot(dx, dy)

        # 找到最近路径点的下标
        target_idx = np.argmin(d)
        self.target_idx = target_idx
        print(f"追踪目标id{self.target_idx}")

        # 按照轨迹顺序追踪(不可行 追踪点快速切换但是小车跟不上)
        # target_idx = self.target_idx
        # self.target_idx += 1

        # 计算横向误差 Crosstrack error
        front_axle_vec = [np.cos(self.yaw - np.pi / 2), np.sin(self.yaw - np.pi / 2)]
        self.crosstrack_error = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        # 计算航向角误差 Heading error 
        self.heading_error = normalise_angle(self.cyaw[target_idx] - self.yaw)
        
        # 绘制追踪点
        tracking_point = PointStamped()
        tracking_point.header.frame_id = "map"
        tracking_point.header.stamp = self.get_clock().now().to_msg()
        tracking_point.point.x = self.cx[target_idx]
        tracking_point.point.y = self.cy[target_idx]
        tracking_point.point.z = 0.0
        self.Traking_point_pub.publish(tracking_point)

        # 计算追踪速度(区分前行/倒车)
        # vector_ve2tg = [-dx[target_idx], -dy[target_idx]]
        # vector_vehicle = [np.cos(self.yaw), np.sin(self.yaw)]
        # # # angle_vector_c2t = normalise_angle(np.arctan2(vector_ve2tg[1], vector_ve2tg[0]))
        # angle_ve2tg = np.arccos(np.dot(vector_ve2tg, vector_vehicle) / (np.linalg.norm(vector_ve2tg) * np.linalg.norm(vector_vehicle)))
        # print(f"轨迹点下标{self.target_idx},与轨迹点方向夹角{round(rad2deg(angle_ve2tg),2)}")
        # if angle_ve2tg > np.pi/2 or angle_ve2tg < -np.pi/2:
        #     self.target_vel = -0.16
        # else:
        #     self.target_vel = 0.16

        # # 删除已追踪过的轨迹点
        # self.cx.pop(self.target_idx)
        # self.cy.pop(self.target_idx)
        # self.cyaw.pop(self.target_idx)

    # Stanley controller determines the appropriate steering angle
    def stanley_control(self):
        self.lock.acquire()
        # 计算待转角
        crosstrack_term = np.arctan2((self.k * self.crosstrack_error), (self.ksoft + self.target_vel))
        sigma_t = crosstrack_term + self.heading_error

        # Constrains steering angle to the vehicle limits
        if sigma_t >= self.max_steer:
            sigma_t = self.max_steer

        elif sigma_t <= -self.max_steer:
            sigma_t = -self.max_steer

        # print(f"航向误差={rad2deg(self.heading_error)}, 偏离误差 ={rad2deg(crosstrack_term)}, 待转角={round(rad2deg(sigma_t),3)}, 速度={self.target_vel}")
        # print(f"航向误差={rad2deg(self.heading_error)}, 偏离误差 ={rad2deg(crosstrack_term)}, 待转角={round(sigma_t,3)}")
        # 下发控制
        if sigma_t and not self.stop:
            self.target_vel = 0.1
            self.set_vehicle_command(self.target_vel, sigma_t)
        self.lock.release()

    # Publishes to vehicle state
    def set_vehicle_command(self, velocity, steering_angle):
        print(f"速度{velocity}, 待转角{steering_angle}")
        ''' Publishes the calculated steering angle  '''
        drive = Twist()
        drive.linear.x = velocity
        drive.linear.y = 0.0
        drive.linear.z = 0.0

        drive.angular.x = 0.0
        drive.angular.y = 0.0
        drive.angular.z = steering_angle

        self.tracker_pub.publish(drive)

def main(args=None):
    rclpy.init(args=args)             # 初始化rclpy
    node = Tracker('Stanley_Trake_node')  # 新建一个节点
    rclpy.spin(node)                  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown()                  # 关闭rclpy

if __name__ == '__main__':
    main()

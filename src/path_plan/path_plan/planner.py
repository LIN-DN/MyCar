#!/usr/bin/env python3
'''
    路径规划器
'''
from .hybridAStar import *
from .obstacleMap import map

import csv
import asyncio
import numpy as np
import threading
import multiprocessing
from numpy import round, rad2deg, deg2rad
from transforms3d.euler import quat2euler, euler2quat

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker
np.set_printoptions(suppress=True)


class Planner(Node):

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} node has been created!")

        self.source_point = [0.0, 0.0, 0.0]
        self.goal_point = [0.0, 0.0, 0.0]
        self.busy = False
        self.hybrid_xyresolution = 4
        self.hybrid_yawresolution = 5.0
        self.gridCellSize = 0.05
        self.origin_posx = 0.0
        self.origin_posy = 0.0

        self.car = Marker()
        self.car.header.frame_id = "map"
        # self.car.header.frame_id = "base_link"
        self.car.type = Marker.CUBE
        self.car.action = Marker.ADD
        self.car.scale.x = 0.4 
        self.car.scale.y = 0.2
        self.car.scale.z = 0.2
        self.car.color.r = 1.0
        self.car.color.g = 0.0
        self.car.color.b = 0.0
        self.car.color.a = 1.0

        # # 静态地图发布
        self.drawMap()
        self.occupancyGrid_pub = self.create_publisher(OccupancyGrid, '/obstacle_map', 10)
        self.map_timer = self.create_timer(1, self.GridMap_Pub) 

        # self.occupancyGrid_sub = self.create_subscription(OccupancyGrid, '/map', self.lidar_map_callback, 1)
        # self.sPose_sub = self.create_subscription(PoseStamped, '/car1/pose', self.current_pose_callback, 1)
        # self.sPose_sub = self.create_subscription(PoseStamped, '/tracked_pose', self.current_pose_callback, 1)
        self.sPose_sub = self.create_subscription(PoseStamped, 'Fused/pose', self.current_pose_callback, 1)
        # self.sPose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.source_pose_callback, 1)
        self.gPose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 1)
        self.path_pub = self.create_publisher(Path, '/Trajectory', 1)
        # self.car_pub = self.create_publisher(Marker, '/simulate_car', 1)

        # 线程锁
        self.lock = threading.Lock()

    def drawMap(self):
        # Get Obstacle Map
        self.obstacleX_Grid, self.obstacleY_Grid = map()

        # Calculate map Paramaters
        self.obstacleX = [x for x in self.obstacleX_Grid]
        self.obstacleY = [y for y in self.obstacleY_Grid]
        self.mapParameters = calculateMapParameters(self.obstacleX, self.obstacleY, self.hybrid_xyresolution, np.deg2rad(self.hybrid_yawresolution))

        # Create OccupancyGrid message
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = self.gridCellSize
        self.map_msg.info.width = int(max(self.obstacleX_Grid))+1
        self.map_msg.info.height = int(max(self.obstacleY_Grid))+1
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0
        
        # Fill in the OccupancyGrid data array based on obstacle locations
        self.map_msg.data = [0] * (self.map_msg.info.width * self.map_msg.info.height)
        # print(f"{int(max(self.obstacleX_Grid)+1)}, {int(max(self.obstacleY_Grid)+1)}, {len(self.map_msg.data)}")
        for i in range(len(self.obstacleX_Grid)):
            index = int(self.obstacleY_Grid[i] * self.map_msg.info.width + self.obstacleX_Grid[i])
            self.map_msg.data[index] = 100

    def GridMap_Pub(self):
        # 发布栅格地图
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.occupancyGrid_pub.publish(self.map_msg)
    
    def lidar_map_callback(self, msg):
        if not self.busy: 
            # 获取地图的宽度和高度
            width = msg.info.width
            height = msg.info.height
            self.origin_posx = msg.info.origin.position.x
            self.origin_posy = msg.info.origin.position.y
            # print(f"原点偏移量{self.origin_posx, self.origin_posy}")
            # 将数据转换为numpy数组，便于处理
            grid_data = np.array(msg.data).reshape((height, width))

            # 初始化障碍物的横坐标列表和纵坐标列表
            obstacles_x = []
            obstacles_y = []

            # 遍历地图，找到障碍物的坐标
            for y in range(height):
                for x in range(width):
                    # 如果当前单元格的值大于某个阈值，则认为是障碍物
                    if grid_data[y, x] > 50:  # 这里的50是一个示例阈值，根据具体情况调整
                        obstacles_x.append(x)
                        obstacles_y.append(y)
            
            # # 打印障碍物的坐标列表
            # print("Obstacles X coordinates:", obstacles_x)
            # print("Obstacles Y coordinates:", obstacles_y)

            self.mapParameters = calculateMapParameters(obstacles_x, obstacles_y, self.hybrid_xyresolution, np.deg2rad(self.hybrid_yawresolution))

    def current_pose_callback(self, msg):
        if not self.busy: 
            # Process the received current Fuesd Pose 
            pose = msg.pose
            self.source_point[0] = (pose.position.x - self.origin_posx) /0.05
            self.source_point[1] = (pose.position.y - self.origin_posy) /0.05
            self.source_point[2] = quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])[2]
            # print(f"当前位置{np.round([self.source_point[0] * 0.05, self.source_point[1] * 0.05],3)}, 偏航{round(rad2deg(self.source_point[2]), 3)}")
            # print(f"当前网格坐标{np.round([self.source_point[0], self.source_point[1]],3)}, 偏航{round(rad2deg(self.source_point[2]), 3)}")
            # print(f"当前真实坐标{np.round([pose.position.x, pose.position.y],3)}, 偏航{round(rad2deg(self.source_point[2]), 3)}")

            # 绘制小车
            # self.car.pose.position = pose.position
            # self.car.pose.orientation = pose.orientation           
            # self.car_pub.publish(self.car)

    def source_pose_callback(self, msg):
        if not self.busy: 
            # Process the received 2D Pose Estimate
            pose = msg.pose.pose
            self.source_point[0] = pose.position.x /0.05
            self.source_point[1] = pose.position.y /0.05
            self.source_point[2] = quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])[2]
            print(f"起点位置{np.round([self.source_point[0] * 0.05, self.source_point[1] * 0.05],3)}, 偏航{round(rad2deg(self.source_point[2]), 3)}")

    def goal_pose_callback(self, msg):
        if not self.busy: 
            # Process the received 2D Goal Pose
            pose = msg.pose
            self.goal_point[0] = (pose.position.x - self.origin_posx) /0.05
            self.goal_point[1] = (pose.position.y - self.origin_posy) /0.05
            self.goal_point[2] = quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])[2]
            # print(f"终点位置{np.round([self.goal_point[0], self.goal_point[1]], 3)}, 偏航{round(rad2deg(self.goal_point[2]), 3)}")
            print(f"终点网格坐标{np.round([self.goal_point[0], self.goal_point[1]],3)}, 偏航{round(rad2deg(self.goal_point[2]), 3)}")
            print(f"终点真实坐标{np.round([pose.position.x, pose.position.y], 3)}, 偏航{round(rad2deg(self.goal_point[2]), 3)}")

            self.path_plan()

    def path_plan(self):

        # Run Hybrid A*
        self.busy = True
        try:
            # 创建路径规划线程
            self.path_queue = multiprocessing.Queue()
            self.hybridAStar_thread = multiprocessing.Process(target=self.run_with_timeout, args=(self.source_point, self.goal_point, self.mapParameters,self.path_queue))
            self.hybridAStar_thread.start()

            # 等待路径规划线程结束
            self.hybridAStar_thread.join(timeout=10)

            # Check if the process is still alive
            if self.hybridAStar_thread.is_alive():
                self.hybridAStar_thread.terminate()
                raise TimeoutError()

            result = self.path_queue.get()
            if result is not None:
                self.path_x, self.path_y, self.path_yaw = result
                self.drawPath()

        except TimeoutError: 
            print("路径规划超时，请重新选择可达终点！")
            self.busy = False

    def run_with_timeout(self, source_point, goal_point, mapParameters,result_queue:multiprocessing.Queue):
        try:
            # Run the path planning function and return the result
            result = run(source_point, goal_point, mapParameters)
            result_queue.put(result)
        except TypeError:
            print("路径规划失败，请确保起点与终点可达！")
            self.busy = False
            result_queue.put(None)

        return result
    
    def drawPath(self):
        # 发布轨迹
        Trajectory = Path()
        Trajectory.header.stamp = self.get_clock().now().to_msg()
        Trajectory.header.frame_id = 'map'
        # Trajectory.header.frame_id = 'base_link'

        for i in range(0,len(self.path_x),1):
            msg = PoseStamped()
            msg.pose.position.x = (self.path_x[i]) * 0.05 + self.origin_posx
            msg.pose.position.y = (self.path_y[i]) * 0.05 + self.origin_posy
            print(f"轨迹途径点{msg.pose.position.x, msg.pose.position.y}偏航角{rad2deg(self.path_yaw[i])}")

            Quat = euler2quat(0.0, 0.0, self.path_yaw[i])
            msg.pose.orientation.w = Quat[0]
            msg.pose.orientation.x = Quat[1]
            msg.pose.orientation.y = Quat[2]
            msg.pose.orientation.z = Quat[3]

            Trajectory.poses.append(msg)

        self.path_pub.publish(Trajectory)
        self.busy = False

        # 绘制小车
        # self.Status_Index = 0
        # self.car_timer = self.create_timer(0.05, self.drawCar)  # 创建一个0.1秒的定时器

    def drawCar(self):
        if self.Status_Index < len(self.path_x):
            i = self.Status_Index
            car = Marker()
            car.header.frame_id = "map"
            car.ns = ""
            car.type = Marker.CUBE
            car.action = Marker.ADD
            car.pose.position.x = self.path_x[i] * 0.05
            car.pose.position.y = self.path_y[i] * 0.05
            Quat = euler2quat(0.0, 0.0, self.path_yaw[i])
            car.pose.orientation.w = Quat[0]
            car.pose.orientation.x = Quat[1]
            car.pose.orientation.y = Quat[2]
            car.pose.orientation.z = Quat[3]            
            car.scale.x = 0.35 
            car.scale.y = 0.25
            car.color.r = 1.0
            car.color.g = 0.0
            car.color.b = 0.0
            car.color.a = 1.0
        
            # 发布Marker数组消息
            self.car_pub.publish(car)

            self.Status_Index += 1
            if self.Status_Index >= len(self.path_x):
                self.busy = False


def main(args=None):
    rclpy.init(args=args)             # 初始化rclpy
    node = Planner('Path_Plan_node')  # 新建一个节点
    rclpy.spin(node)                  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown()                  # 关闭rclpy

if __name__ == '__main__':
    main()
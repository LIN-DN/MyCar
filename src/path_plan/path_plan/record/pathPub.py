#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

class PathRecord(Node):

    def __init__(self):
        super().__init__('path_record')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # 根据需要调整深度
            lifespan=Duration(seconds=5)
        )

        # Initialise subscribers
        self.path_pub = self.create_publisher(Path, '/record_path', qos_profile)

        paths = self.read_file()
        self.pub_path(paths)

    def pub_path(self,paths):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, len(paths)):
            # Appending to Visualization Path
            point = PoseStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.pose.position.x = paths[i][0]
            point.pose.position.y = paths[i][1]
            point.pose.position.z = 0.0
            point.pose.orientation.w = paths[i][2]
            point.pose.orientation.x = paths[i][3]
            point.pose.orientation.y = paths[i][4]
            point.pose.orientation.z = paths[i][5]

            path.poses.append(point)
        self.path_pub.publish(path)
        print(f"发布数据\n{paths}")

    def read_file(self):
        """
        读取路径记录文件
        """
        path_list = []
        # record_file = '/home/cat/agv_ws/src/path_plan/path_plan/path/'+'cefang.csv'
        record_file = '/home/cat/agv_ws/src/path_plan/path_plan/path/'+'daoche.csv'
        # file_path = os.path.join(get_package_share_directory('path_plan'), 'record', '1.csv')
        with open(record_file, 'r') as file:
            for line in file:
                # 去除换行符并按逗号分割字符串
                point = line.strip().split(',')
                # 将字符串转换为浮点数并添加到路径列表中
                path_list.append([float(point[0]), float(point[1]),float(point[2]),float(point[3]),float(point[4]),float(point[5])])
            return path_list


def main(args=None):
    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        path_record = PathRecord()
        # Stop the node from exiting
        rclpy.spin(path_record)

    finally:
        # path_tracker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import math
import os
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class PathRecord(Node):

    def __init__(self):

        super().__init__('path_record')
        # Initialise subscribers
        # self.localisation_sub = self.create_subscription(PoseStamped, '/car0/pose', self.vehicle_state_cb, 1)
        # self.localisation_sub = self.create_subscription(PoseStamped, '/tracked_pose', self.vehicle_state_cb, 1)
        self.localisation_sub = self.create_subscription(PoseStamped, 'Fused/pose', self.vehicle_state_cb, 1)

        self.old_pose = None
        self.path = []

    def save_path_to_file(self):
        """
        Saves a path to a file.
        将路径保存到文件中。
        """
        # record_file = '/home/cat/agv_ws/src/path_plan/path_plan/path/'+'daoche1.csv'
        record_file = '/home/cat/agv_ws/src/path_plan/path_plan/path/'+'cefang1.csv'
        # file_path = os.path.join(get_package_share_directory('path_plan'), 'record', '1.csv')
        # print(f"file_path={file_path}")
        with open(record_file, 'w') as file:
            for point in self.path:
                file.write(f"{point[0]},{point[1]},{point[2]},{point[3]},{point[4]},{point[5]}\n")

    def vehicle_state_cb(self, msg):
        """
        更新车辆状态
        """
        px = msg.pose.position.x
        py = msg.pose.position.y
        # theta = quaternion_to_theta(msg.pose.orientation)
        ow = msg.pose.orientation.w
        ox = msg.pose.orientation.x
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z

        # raw = PathRecord.quat_to_yaw(msg.pose.orientation)

        # offset = 0.10

        # px = px - offset * np.cos(raw)
        # py = py - offset * np.sin(raw)

        if not self.old_pose:
            self.old_pose = [px,py]
            self.path.append([px,py,ow,ox,oy,oz])
            print(f"point={[px,py,ow,ox,oy,oz]}")

        else:
            distance = np.linalg.norm(np.array(self.old_pose) - np.array([px,py]))
            # print(f"distance={distance}")
            if distance>=0.01:
                print(f"point={[px,py,ow,ox,oy,oz]}")
                self.path.append([px,py,ow,ox,oy,oz])
                self.old_pose = [px,py]

    @staticmethod
    def quat_to_yaw(quater):
         
        w = quater.w
        x = quater.x
        y = quater.y
        z = quater.z

        raw = np.arctan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw


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
        path_record.save_path_to_file()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
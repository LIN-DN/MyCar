import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    t1 = Node(package = "car_2_sonic_obstacle",executable="sonic_obstacle",name="t1")
    t2 = Node(package = "car_4_communication",executable="net_work",name="t2")
    t3 = Node(package = "car_7_path_planner",executable="global_path_planning",name="t3")
    t4 = Node(package = "chassis_control",executable="rfid_read",name="t4")
    t5 = Node(package = "chassis_control",executable="chassis_write",name="t5")
    t6 = Node(package = "chassis_control",executable="car_run",name="t6")
   
    

    return LaunchDescription([t1,t2,t3,t4,t5,t6])

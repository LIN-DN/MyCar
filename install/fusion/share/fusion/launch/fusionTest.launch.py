# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    # 创建Actions.Node对象，标明节点所属功能包(节点名与setup.py中对应)
    imu_node = Node(
        package="chassis_control",
        executable="car_motion_control"
        )
    UWB_distance = Node(
        package="fusion",
        executable="UWB_distance"
        )
    fusion_location = Node(
        package="fusion",
        executable="fusion_location"
        )
    fusion_Path = Node(
        package="fusion",
        executable="fusion_Path"
        )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([imu_node, UWB_distance,fusion_location,fusion_Path])
    # 返回让ROS2根据launch描述执行节点
    return launch_description

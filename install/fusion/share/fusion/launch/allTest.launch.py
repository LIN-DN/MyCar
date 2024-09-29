# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    imu_node = Node(
        package="chassis_control",
        executable="car_motion_control"
        )
    UWB_distance = Node(
        package="fusion",
        executable="UWB_distance"
        )
    uwb_Path = Node(
        package="fusion",
        executable="uwb_Path"
        )
    fusion_location = Node(
        package="fusion",
        executable="fusion_location"
        )
    fusion_Path = Node(
        package="fusion",
        executable="fusion_Path"
        )
    mocap_Path = Node(
        package="fusion",
        executable="mocap_Path"
        )
    Error_node = Node(
        package="fusion",
        executable="Error_node"
        )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([imu_node, UWB_distance, uwb_Path, fusion_location, fusion_Path, mocap_Path, Error_node])
    # 返回让ROS2根据launch描述执行节点
    return launch_description

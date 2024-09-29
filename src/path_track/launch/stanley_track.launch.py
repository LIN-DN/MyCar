# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    # 创建Actions.Node对象，标明节点所属功能包(节点名与setup.py中对应)
    path_plan = Node(
        package="path_plan",
        executable="path_planner"
        )
    path_track = Node(
        package="path_track",
        executable="stanley_tracker"
        )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([path_plan, path_track])
    # 返回让ROS2根据launch描述执行节点
    return launch_description

from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
# 事件相关
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
# 获取功能包下share目录路径
# from ament_index_python.packages import get_package_share_directory

'''
    创建三个turtlesim_node节点
    前两个为为一组，第三个为一组
'''

def generate_launch_description():
    t1 = Node(package="turtlesim", executable="turtlesim_node",name="t1")
    t2 = Node(package="turtlesim", executable="turtlesim_node",name="t2")
    t3 = Node(package="turtlesim", executable="turtlesim_node",name="t3")

    g1 = GroupAction(actions=[PushRosNamespace(namespace="g1"), t1, t2])
    g2 = GroupAction(actions=[PushRosNamespace(namespace="g2"), t3])
    return LaunchDescription([g1, g2])
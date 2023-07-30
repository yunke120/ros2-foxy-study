
from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory
import os

'''
    包含 launch 文件 (在当前launch文件包含其他launch文件)
'''

def generate_launch_description():
    launch_file_path = os.path.join(get_package_share_directory("cpp01_launch"), "launch", "py04_args_launch.py")
    include = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(launch_file_path=launch_file_path),
        launch_arguments=[
            ("backg_r","50"),
            ("backg_g","50"),
            ("backg_b","50")
        ]
    )
    return LaunchDescription([include])
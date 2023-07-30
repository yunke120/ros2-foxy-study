
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
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory
import os
'''
    演示 Node 的使用

        .. deprecated:: Foxy
           Parameters `node_executable`, `node_name`, and `node_namespace` are deprecated.
           Use `executable`, `name`, and `namespace` instead.

        :param: executable : 可执行程序
        :param: node_executable (DEPRECATED) the name of the executable to find if a package
            is provided or otherwise a path to the executable to run.
        :param: package : 被执行的程序所属功能包
        :param: name    : 节点名称
        :param: namespace : 节点命名空间
        :param: exec_name : 设置程序标签
        :param: node_name (DEPRECATED) the name of the node
        :param: node_namespace (DEPRECATED) the ros namespace for this Node
        :param: parameters : 设置参数
        :param: remappings : 实现话题重映射
        :param: arguments  : 为节点传参
'''
def generate_launch_description():

    t1 = Node(
        package="turtlesim", 
        executable="turtlesim_node",
        exec_name="my_label",
        name = "t1",
        # 方式一：
        # parameters=[{"background_r": 255,"background_b": 0,"background_g": 222}])
        # 方式二(加载yaml绝对路经)
        # parameters=["/home/ros/rosprj/ws02_tools/install/cpp01_launch/share/cpp01_launch/config/t1.yaml"]
        # 动态获取路经
        parameters=[os.path.join(get_package_share_directory(package_name="cpp01_launch"), "config", "t1.yaml")]
    )

    return LaunchDescription([t1])
    
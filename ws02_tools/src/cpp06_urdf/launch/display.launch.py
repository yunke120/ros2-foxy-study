from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    '''
    加载urdf文件并在rviz中显示机器人模型

    启动 robot_state_publisher节点, 以参数形式加载urdf文件
    启动rviz2

    优化：
        添加 joint_state_publisher节点
        添加 rviz2 默认配置文件
        动态传入urdf文件
    
    '''
    # ros2 launch cpp06_urdf display.launch.py model:='ros2 pkg prefix --share cpp06_urdf'/urdf/urdf/demo01_helloworld2.urdf
    # 'ros2 pkg prefix --share cpp06_urdf' 相当于 get_package_share_directory 但是在这里未起作用,原因是这个指令未执行
    # 直接运行可以
    # ros2 launch cpp06_urdf display.launch.py model:=/home/ros/rosprj/ws02_tools/install/cpp06_urdf/share/cpp06_urdf/urdf/urdf/demo01_helloworld2.urdf
    # 或者
    # ros2 launch cpp06_urdf display.launch.py model:=$(ros2 pkg prefix --share cpp06_urdf)/urdf/urdf/demo01_helloworld2.urdf
    model = DeclareLaunchArgument(name="model", default_value=get_package_share_directory("cpp06_urdf") + "/urdf/urdf/demo01_hellooworld.urdf")

    # p_value = ParameterValue(Command(["xacro ", get_package_share_directory("cpp06_urdf") + "/urdf/urdf/demo01_hellooworld.urdf"]))
    p_value = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description":p_value}]
    )

    # joint_state_pub = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    # )



    rviz2 = Node(
        package='rviz2', 
        executable='rviz2',
        arguments=["-d", get_package_share_directory("cpp06_urdf") + "/rviz/urdf.rviz"]    
    )

    # return LaunchDescription([model, rviz2, robot_state_pub, joint_state_pub])
    return LaunchDescription([model, rviz2, robot_state_pub])


'''
    
    问题描述： 通过joint_state_publisher_gui 让关节运行到指定位置后，关节存在抖动，在初始位置和指定位置之间抖动
    解决：不再启动 joint_state_publisher 节点
    原因：1. joint_state_publisher与joint_state_publisher_gui都会发布非固定关节的运动信息
         2. robot_state_publisher会订阅关节的运动信息，并生成坐标变换数据广播
         3. joint_state_publisher与joint_state_publisher_gui有一个存在时就会发布关节运动信息，进而就能生成坐标变换
            当两个都不启动时，坐标树生成不了，机器人模型显示异常
            当两个都存在时，joint_state_publisher发布初始关节姿态信息
                         joint_state_publisher_gui发布指定位置信息
    运行：
    1. $ ros2 run joint_state_publisher_gui joint_state_publisher_gui
    2. $ ros2 launch cpp06_urdf display.launch.py model:=$(ros2 pkg prefix --share cpp06_urdf)/urdf/urdf/demo03_joint.urdf

'''
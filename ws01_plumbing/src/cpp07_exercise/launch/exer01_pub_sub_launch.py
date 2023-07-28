
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # 1. 启动两个 turtlesim_node，其中一个要设置命名空间
    t1 = Node(package="turtlesim", executable="turtlesim_node")
    t2 = Node(package="turtlesim", executable="turtlesim_node", namespace="t2")

    # 2. 控制第二个乌龟掉头
    rotate = ExecuteProcess(
        cmd=["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        output="both",
        shell=True
    )

    # 3. 调用自定义节点
    exer01 = Node(package="cpp07_exercise", executable="exer01_pub_sub")
    # 怎么控制节点的执行顺序，通过注册事件来完成
    # 要创建事件注册对象，在对象当中声明针对哪个目标节点，在事件触发时执行哪种操作
    register_rotate_exit_event = RegisterEventHandler(
        # 创建一个新对象
        event_handler=OnProcessExit(    # 触发动作
            target_action=rotate,       # 目标节点
            on_exit=exer01              # 触发执行的事件
        )   # 目标节点的事件触发某个动
    )

    return LaunchDescription([t1,t2,rotate,register_rotate_exit_event])
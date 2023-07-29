
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():

    x = 9.54
    y = 9.54
    theta = 1.0

    name = "t2"
    # ros2 service call /spawn turtlesim/srv/Spawn "{'x': 6.0, 'y': 9.0, 'theta': 0.0, 'name': 't2'}"

    spwan = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': " 
             + str(x) + ", 'y': " + str(y) 
             + ", 'theta': "+ str(theta)
             + ", 'name': " + name + "}\""],
        output="both",
        shell=True)

    client = Node(package="cpp07_exercise", 
                  executable="exer05_action_client",
                  arguments=[str(x), str(y), str(theta)])
    # 相当于在终端输入 ros2 run cpp07_exercise exer02_client x y theta --ros-args
    # 在目标点生成一只新乌龟

    # 调用客户端发送目标点坐标

    return LaunchDescription([spwan, client])
    
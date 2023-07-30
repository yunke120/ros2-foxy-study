
# ROS2 Foxy Study
来源：赵虚左老师的[ROS2理论与实践](https://www.bilibili.com/video/BV1VB4y137ys)

## 实践平台
- Jetson Xavier Nx
- JetPack 5.0.2 (Ubuntu 20.04) [参考](https://www.stereolabs.com/blog/nvidia-jetson-l4t-and-jetpack-support/)
- Ros2 (foxy)

## 说明
1. ws01_plumbing : 通信机制与实践
2. ws02_tools : ROS2常用工具使用

## VSCode 代码片段
**python代码片段**
```json
{
	"ros2 node python": {
		"prefix": "ros2_node_python",
		"body": [
		"import rclpy",
		"from rclpy.node import Node",
		"",
		"",
		"class MyNode(Node):",
		"    def __init__(self):",
		"        super().__init__(\"MyNode_py\")",
		"",
		"",
		"def main():",
		"    rclpy.init()",
		"    rclpy.spin(MyNode())",
		"    rclpy.shutdown()",
		"",
		"",
		"if __name__ == '__main__':",
		"    main()",
		],
		"description": "ros2 node"
	},
	"ros2 launch py": {
		"prefix": "ros2_launch_py",
		"body": [
		"from launch import LaunchDescription",
		"from launch_ros.actions import Node",
		"# 封装终端指令相关类",
		"# from launch.actions import ExecuteProcess",
		"# from launch.substitutions import FindExecutable",
		"# 参数声明与获取",
		"# from launch.actions import DeclareLaunchArgument",
		"# from launch.substitutions import LaunchConfiguration",
		"# 文件包含相关",
		"# from launch.actions import IncludeLaunchDescription",
		"# from launch.launch_description_sources import PythonLaunchDescriptionSource",
		"# 分组相关",
		"# from launch_ros.actions import PushRosNamespace",
		"# from launch.actions import GroupAction",
		"# 事件相关",
		"# from launch.event_handlers import OnProcessStart, OnProcessExit",
		"# from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo",
		"# 获取功能包下share目录路径",
		"# from ament_index_python.packages import get_package_share_directory",
		"",
		"def generate_launch_description():",
		"",
		"    return LaunchDescription([])",
		],
		"description": "ros2 launch py"
	}
}
```
**cpp代码片段**
```json
{
	"ros2 node cpp": {
		"prefix": "ros2_node_cpp",
		"body": [
			"#include \"rclcpp/rclcpp.hpp\"",
			"",
			"",
			"class MyNode: public rclcpp::Node{",
			"public:",
			"    MyNode():Node(\"MyNode_cpp\"){",
			"       ",	
			"    }",
			"",
			"};",
			"",
			"",
			"int main(int argc, char ** argv)",
			"{",
			"    rclcpp::init(argc, argv);",
			"    rclcpp::spin(std::make_shared<MyNode>());",
			"    rclcpp::shutdown();",
			"    return 0;",
			"}",
		],
		"description": "ros2 node"
	}
}
```



## 学习资源
1. [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)





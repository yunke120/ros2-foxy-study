

import rclpy
from rclpy.node import Node
class MyParam(Node):
    def __init__(self):
        super().__init__("my_param_node_py")
        self.get_logger().info("参数API使用 Python")

        p1 = rclpy.Parameter("car_name", value="tiger")
        p2 = rclpy.Parameter("height", value=1.5)
        p3 = rclpy.Parameter("wheels", value=4)

        self.get_logger().info("car_name = %s" % p1.value)
        self.get_logger().info("height = %s" % p2.value)
        self.get_logger().info("wheels = %s" % p3.value)

        self.get_logger().info("key = %s" % p1.name)
        # self.get_logger().info("key = %s" % p1.name)
        # self.get_logger().info("key = %s" % p1.name)


def main():
    rclpy.init()
    rclpy.spin(MyParam())
    rclpy.shutdown()
if __name__ == '__main__':
    main()

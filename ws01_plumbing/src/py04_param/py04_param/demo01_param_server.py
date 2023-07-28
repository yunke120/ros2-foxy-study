

import rclpy
from rclpy.node import Node


class MyParamServer(Node):
    def __init__(self):
        # 如果要允许删除参数，需要提前声明
        super().__init__("param_server_py", allow_undeclared_parameters=True)
        self.get_logger().info("参数服务端 Python")

    def delcare_param(self):
        self.get_logger().info("---------- 增 ----------")
        self.declare_parameter("car_name", "tiger")
        self.declare_parameter("height", 3.45)
        self.declare_parameter("wheels", 4)

        pass

    def get_param(self):
        self.get_logger().info("---------- 查 ----------")
        car = self.get_parameter("car_name")
        self.get_logger().info("(%s = %s)" % (car.name, car.value))

        params = self.get_parameters(["car_name", "wheels"])
        for p in params:
            self.get_logger().info("(%s = %s)" % (p.name, p.value))
        pass

    def update_param(self):
        self.get_logger().info("---------- 改 ----------")
        self.set_parameters([rclpy.Parameter("car_name", value="hello")])
        pass

    def del_param(self):
        self.get_logger().info("---------- 删 ----------")
        self.undeclare_parameter("car_name")
        pass




def main():
    rclpy.init()
    node = MyParamServer()
    node.delcare_param()
    node.get_param()
    node.update_param()
    node.get_param()
    node.del_param()
    node.get_param()

    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
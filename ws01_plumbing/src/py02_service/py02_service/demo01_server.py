
import rclpy
from rclpy.node import Node

from base_interfaces_demo.srv import AddInts

'''
1. 创建服务端
2. 编写回调函数

'''


class AddIntsServer(Node):
    def __init__(self):
        super().__init__("addints_server_py")
        self.get_logger().info("创建服务端 Python")

        self.server = self.create_service(AddInts, "add_ints", self.add)

    def add(self, req, res):
        res.sum = req.num1 + req.num2
        self.get_logger().info("%d + %d = %d" % (req.num1, req.num2, res.sum))
        return res
        



def main():
    rclpy.init()
    rclpy.spin(AddIntsServer())
    rclpy.shutdown()





if __name__ == '__main__':
    main()


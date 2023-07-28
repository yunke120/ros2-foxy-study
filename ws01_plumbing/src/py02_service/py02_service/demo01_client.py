

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from base_interfaces_demo.srv import AddInts
import sys
'''
    编写客户端实现，提交两个整形数据并处理响应结果

'''
class AddIntsClient(Node):
    def __init__(self):
        super().__init__("addints_client_py")
        self.get_logger().info("创建客户端 Python")

        self.client = self.create_client(AddInts, "add_ints")

        while not self.client.wait_for_service(1.0):
            self.get_logger().info("服务连接中...")
        self.get_logger().info("服务连接成功")
        

    def send_request(self):
        request = AddInts.Request()
        request.num1 = int(sys.argv[1])
        request.num2 = int(sys.argv[2])

        self.future = self.client.call_async(request)
        
def main():
    # 校验输入参数数量
    if len(sys.argv) != 3:
        get_logger("rclpy").error("清提交两个整形数据")
        return

    rclpy.init()
    client = AddIntsClient()
    # 发送请求
    client.send_request()
    rclpy.spin_until_future_complete(client, client.future)
    try:
        response = client.future.result()
        client.get_logger().info("响应结果, sum = %d" % response.sum)
    except Exception:
        client.get_logger().error("服务响应失败")

    client.send_request()
    rclpy.spin_until_future_complete(client, client.future)
    try:
        response = client.future.result()
        client.get_logger().info("响应结果, sum = %d" % response.sum)
    except Exception:
        client.get_logger().error("服务响应失败")
    rclpy.shutdown()

if __name__ == '__main__':
    main()





import rclpy
from rclpy.node import Node

from base_interfaces_demo.msg import Student

class ListenerStu(Node):
    def __init__(self):
        super().__init__("listener_stu_py")

        self.subscription = self.create_subscription(Student, "chatter_stu", self.on_cb, 10)

    def on_cb(self, msg):
        self.get_logger().warn("接收消息: %s %d %.2f" % (msg.name, msg.age, msg.height))

def main():
    rclpy.init()
    rclpy.spin(ListenerStu())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
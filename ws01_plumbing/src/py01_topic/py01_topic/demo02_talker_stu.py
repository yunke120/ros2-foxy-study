

import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

class TalkerStu(Node):
    def __init__(self):
        super().__init__("talker_stu_py")
        self.get_logger().info("创建发布者 Python")
        self.publisher = self.create_publisher(Student, "chatter_stu", 10)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        msg = Student()
        msg.name = "hahaha"
        msg.age = 20
        msg.height = 21.5
        self.publisher.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TalkerStu())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
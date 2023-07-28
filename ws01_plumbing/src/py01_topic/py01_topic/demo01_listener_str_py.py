
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__("node_listener_python")
        self.get_logger().info("订阅节点创建(Python)")
        self.subscription_ = self.create_subscription(String, "chatter", self.on_cb, 10)

    def on_cb(self, msg):

        self.get_logger().info(msg.data)


def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()


if __name__ == '__main__':
    main()


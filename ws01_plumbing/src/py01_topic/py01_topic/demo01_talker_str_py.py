
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self) -> None:
        super().__init__("talker_node_py")
        self.get_logger().info("发布方创建节点(Py)")
    
        self.publisher = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.count = 0
    def on_timer(self):
        msg = String()
        msg.data = "hello world(py)" + str(self.count)
        self.publisher.publish(msg)
        self.count += 1
        self.get_logger().info("发布的数据: %s" % msg.data)

def main():
    
    rclpy.init()
    rclpy.spin(Talker())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

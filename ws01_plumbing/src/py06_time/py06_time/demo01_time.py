

import rclpy
from rclpy.node import Node
import time
import threading

from rclpy.time import Time, Duration

class MyNode(Node):
    def __init__(self):
        super().__init__("time_node_py")
        # self.demo_rate()
        # self.demo_time()
        # self.demo_duration()
        self.demo_opt()

    def demo_rate(self):
        self.rate = self.create_rate(1.0)

        thread = threading.Thread(target=self.do_some)
        thread.start()

    def do_some(self):
        while rclpy.ok():
            self.get_logger().info("hello")
            self.rate.sleep()             # 会导致程序阻塞，不能放在主线程
            # time.sleep(1)
    
    def demo_time(self):
        self.time1 = Time(seconds=5, nanoseconds=500000000)
        self.time2 = self.get_clock().now()

        self.get_logger().info("s = %.2f, ns = %d" % (self.time1.seconds_nanoseconds()[0], self.time1.seconds_nanoseconds()[1]))
        self.get_logger().info("s = %.2f, ns = %d" % (self.time2.seconds_nanoseconds()[0], self.time2.seconds_nanoseconds()[1]))

    def demo_duration(self):
        du1 = Duration(seconds=1, nanoseconds=50000000)
        self.get_logger().info("ns = %d" % du1.nanoseconds)

    def demo_opt(self):
        t1 = Time(seconds=20)
        t2 = Time(seconds=15)

        d1 = Duration(seconds=7)
        d2 = Duration(seconds=13)


def main():
    rclpy.init()
    rclpy.spin(MyNode())
    rclpy.shutdown()



if __name__ == '__main__':
    main()
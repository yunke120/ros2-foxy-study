
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from base_interfaces_demo.action import Progress

class ProgressActionClient(Node):
    def __init__(self):
        super().__init__("progressactionclient")
        self.get_logger().info("动作客户端")
        
        # 创建动作客户端
        '''
            (node: Any, 
            action_type: Any, 
            action_name: Any, 
            *, 
            callback_group: Any | None = None, 
            goal_service_qos_profile: QoSProfile = qos_profile_services_default, 
            result_service_qos_profile: QoSProfile = qos_profile_services_default, 
            cancel_service_qos_profile: QoSProfile = qos_profile_services_default, 
            feedback_sub_qos_profile: QoSProfile = QoSProfile(depth=10), 
            status_sub_qos_profile: QoSProfile = qos_profile_action_status_default) -> None
        '''
        self.client = ActionClient(self, Progress, "get_sum")

    def send_goal(self, num):
        if not self.client.wait_for_server(10.0):
            return
        goal = Progress.Goal()
        goal.num = num
        self.future = self.client.send_goal_async(goal, self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, fb_msg):
        progress = fb_msg.feedback.progress
        self.get_logger().info("连续反馈数据: %.2f" % progress)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        # print(type(goal_handle))  rclpy.action.client.ClientGoalHandle
        self.get_logger().info(goal_handle.__str__())
        if not goal_handle.accepted:
            self.get_logger().error("目标被拒绝")
            return
        self.get_logger().info("目标被接收")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("最终响应结果: %d" % result.sum)
def main():
    if len(sys.argv) != 2:
        get_logger("rclpy").error("请提交一个整形数据")
        return

    rclpy.init()
    client_ = ProgressActionClient()
    client_.send_goal(int(sys.argv[1]))
    rclpy.spin(client_)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
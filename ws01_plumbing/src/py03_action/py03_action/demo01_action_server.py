import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time
from base_interfaces_demo.action import Progress
from rclpy.action import GoalResponse, CancelResponse
class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("progressactionserver")
        self.get_logger().info("动作服务端")
        self.cancel_flag = False
        # 创建动作客户端
        '''
        node: Any, 
        action_type: Any, 
        action_name: Any, 
        execute_callback: Any, *, 
        callback_group: Any | None = None, 
        goal_callback: Any = default_goal_callback, 
        handle_accepted_callback: Any = default_handle_accepted_callback, 
        cancel_callback: Any = default_cancel_callback
        '''
        self.server = ActionServer(
            self,
            Progress,
            "get_sum",
            self.execute_callback,
            cancel_callback = self.cancel_callback,
            goal_callback = self.goal_callback,
        )


    def execute_callback(self, goal_handle):
        # 1. 生成连续反馈
        num = goal_handle.request.num
        result = Progress.Result()
        sum = 0
        for i in range(1, num+1):
            sum += i
            feedback = Progress.Feedback()
            feedback.progress = i / num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("当前进度：%.2f" % feedback.progress)

            if goal_handle.is_cancel_requested :
                goal_handle.canceled()
                result.sum = sum
                self.get_logger().info("任务被取消")
                return sum
            
            time.sleep(1.0)
        # 2. 响应最终结果
        goal_handle.succeed()
        
        result.sum = sum
        self.get_logger().info("最终结果: %d" % result.sum)
        return result


    def goal_callback(self, goal_request):
        if goal_request.num <= 1:
            self.get_logger().error("提交的目标值非法")
            return GoalResponse.REJECT
        else:
            self.get_logger().info("提交的目标值合法")
            return  GoalResponse.ACCEPT 

    def cancel_callback(self, cancel_request):
        self.get_logger().info("接收到任务取消请求")
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    rclpy.spin(ProgressActionServer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
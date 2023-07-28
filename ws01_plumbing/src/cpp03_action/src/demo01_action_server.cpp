#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

/*
  1. 创建动作服务端对象
  2. 处理提交的目标值
  3. 生成连续反馈
  4. 响应最终结果
  5. 处理取消请求
*/



/*
rclcpp_action::Server<ActionT>::SharedPtr 
            create_server<ActionT, NodeT>
                          (NodeT node, 
                          const std::string &name, 
                          rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
                          rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
                          rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
                          const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
                          rclcpp::CallbackGroup::SharedPtr group = nullptr)

*/
using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionServer: public rclcpp::Node{
public:
    ProgressActionServer():Node("ProgressActionServer"){
      RCLCPP_INFO(this->get_logger(), "创建动作服务端 CPP");

      // 创建对象
      server_ = rclcpp_action::create_server<Progress>(
          this, 
          "get_sum",
          std::bind(&ProgressActionServer::handle_goal, this, _1, _2),
          std::bind(&ProgressActionServer::handle_cancel, this, _1),
          std::bind(&ProgressActionServer::handle_accepted, this, _1)
        );
    }
private:

/*
  /// Signature of a callback that accepts or rejects goal requests.
  using GoalCallback = std::function<GoalResponse(
        const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
  /// Signature of a callback that accepts or rejects requests to cancel a goal.
  using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  /// Signature of a callback that is used to notify when the goal has been accepted.
  using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;

*/
    // 处理提交目标值
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Progress::Goal> goal)
    {
      (void)uuid;
      // 业务逻辑
      // 判断提交的数字是否大于1，是就接收，否就拒绝
      if(goal->num <= 1)
      {
        RCLCPP_ERROR(this->get_logger(), "提交的数据必须大于1");
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(), "提交的目标值合法");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handler) 
    {
        (void)goal_handler;
        RCLCPP_INFO(this->get_logger(), "接收到任务取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        // 1. 生成连续反馈返回给客户端
        // void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
        // goal_handle->publish_feedback()
        // 先获取目标值，然后遍历，遍历中进行累加，且每循环一次就计算进度，并作为连续反馈发布
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        auto feedback = std::make_shared<Progress::Feedback>();
        auto ret = std::make_shared<Progress::Result>();
        // 设置休眠
        rclcpp::Rate rate(1.0);
        for(int i = 1; i <= num; i++)
        {
            sum += i;
            double progress = i / (double)num; // 计算进度
            feedback->progress = progress;

            goal_handle->publish_feedback(feedback);

            // 判断是否接到了取消请求
            if(goal_handle->is_canceling())
            {
                // void canceled(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
                ret->sum = sum;
                goal_handle->canceled(ret);
                RCLCPP_WARN(this->get_logger() , "任务被取消");
                return;
            }
            // void canceled(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
            // goal_handle->canceled()

            rate.sleep();
        }
        // 2. 生成最终响应结果。
        // void succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
        // goal_handle->succeed()

        if(rclcpp::ok())
        {
            
            ret->sum = sum;
            goal_handle->succeed(ret);
        }
    }

    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        // 新建子线程，处理耗时的主逻辑实现
        std::thread(std::bind(&ProgressActionServer::execute, this, goal_handle)).detach();
    }




    rclcpp_action::Server<Progress>::SharedPtr server_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProgressActionServer>());
    rclcpp::shutdown();
    return 0;
}

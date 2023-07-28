#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class ProgressActionClient: public rclcpp::Node{
public:
    ProgressActionClient():Node("ProgressActionClient"){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"创建动作客户端 CPP");

        /**
         * rclcpp_action::Client<ActionT>::SharedPtr 
         * create_client<ActionT, NodeT>(
         * NodeT node, 
         * const std::string &name, 
         * rclcpp::CallbackGroup::SharedPtr group = nullptr)
        */
        client_ = rclcpp_action::create_client<Progress>(this, "get_sum");


    }

    void send_goal(int num)
    {
        // 1. 需要连接服务端
        if(!client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "服务连接失败");
            return;
        }
        // 2. 发送请求
        /**
         * std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr>
         * async_send_goal(
         * const base_interfaces_demo::action::Progress::Goal &goal, 
         * const rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions &options)
         * 
        */
        auto goal = Progress::Goal();
        goal.num = num;
        rclcpp_action::Client<Progress>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback, this, _1);
        // options.goal_response_callback = [this](std::shared_future<rclcpp_action::ClientGoalHandle<Progress>::SharedPtr> future)
        // {
        //     RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
        //     this->goal_handle_ = future.get();
        // };
        // options.goal_response_callback = [this](std::shared_future<rclcpp_action::ClientGoalHandle<Progress>::SharedPtr> future) {
        //     goal_response_callback(future.get());
        // };
        options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&ProgressActionClient::result_callback, this, _1);

        auto future = client_->async_send_goal(goal, options);
    }
    /*
     using GoalHandle = ClientGoalHandle<ActionT>;
    using GoalResponseCallback = std::function<void (std::shared_future<typename GoalHandle::SharedPtr>)>;
    */

   /*
   操作数类型为:  std::function<void (std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>>>)> = 
   std::_Bind<void (ProgressActionClient::*(ProgressActionClient *, std::_Placeholder<1>))
   (std::shared_ptr<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>> goal_handle)>
   */
    // void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle)
    // {

    // }
    // 处理关于目标值的服务端响应
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<Progress>::SharedPtr> goal_handle)
    {
        if(goal_handle.get() == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "目标请求被服务器拒绝");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标处理中");
        }
    }
    /**
     *  std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;

        GoalHandleFibonacci::SharedPtr,const std::shared_ptr<const Fibonacci::Feedback> feedback
    */
    // 处理连续反馈
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle, const std::shared_ptr<const Progress::Feedback> feedback)
    {
        (void)goal_handle;
        double progress = feedback->progress;
        int pro = (int)(progress*100);
        RCLCPP_INFO(this->get_logger(), "当前进度 %d%%", pro);
    }

    /**
     *   using ResultCallback = std::function<void (const WrappedResult & result)>;
    */
    // 处理最终响应
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result)
    {
        // result.code 通过状态码判断最终结果状态
        if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "最终结果为: %d", result.result->sum);
        }
        else if(result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_WARN(this->get_logger(), "被中断!");
        }
        else if(result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "被取消!");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "未知错误!");
        }

    }
private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
};


int main(int argc, char ** argv)
{ 
    if(argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整形数据");
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProgressActionClient>();
    node->send_goal(atoi(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
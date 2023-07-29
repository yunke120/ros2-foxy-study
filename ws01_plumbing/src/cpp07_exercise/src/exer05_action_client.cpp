

#include "rclcpp/rclcpp.hpp"

/**
 * 解析launch 文件传入的参数
 * 
 * 创建动作客户端
 * 连接服务端
 * 向服务端发送请求
 * 处理目标值相关响应结果
 * 处理连续反馈
 * 处理最终响应结果
 * 
*/

#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using base_interfaces_demo::action::Nav;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class Exer05ActionClient: public rclcpp::Node{
public:
    Exer05ActionClient():Node("exer05_action_client_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建动作客户端");
    /* rclcpp_action::Client<ActionT>::SharedPtr 
        create_client<ActionT, NodeT>(
            NodeT node, 
            const std::string &name, 
            rclcpp::CallbackGroup::SharedPtr group = nullptr)
    */
        client_ = rclcpp_action::create_client<Nav>(this, "nav");

    }

    void send_goal(float x, float y, float theta){
        // 连接服务端
        if(!client_->wait_for_action_server(10s))
        {
            RCLCPP_WARN(this->get_logger(), "连接动作服务端超时");
            return;
        }
        // 调用动作客户端发送请求，组织并发布数据
/*
std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>::SharedPtr> 
async_send_goal(
    const base_interfaces_demo::action::Nav::Goal &goal, 
    const rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions &options)
*/
        base_interfaces_demo::action::Nav::Goal goal;
        goal.goal_x = x;
        goal.goal_y = y;
        goal.goal_theta = theta;

        rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions options;
/*
std::function<void (std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>>>)> rclcpp_action::Client<Nav>::SendGoalOptions::goal_response_callback
*/

/*
std::function<void (std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>>, std::shared_ptr<const base_interfaces_demo::action::Nav_Feedback>)> rclcpp_action::Client<Nav>::SendGoalOptions::feedback_callback
*/

/*
std::function<void (const rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>::WrappedResult &result)> rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions::result_callback
*/
        options.goal_response_callback = std::bind(&Exer05ActionClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&Exer05ActionClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&Exer05ActionClient::result_callback, this, _1);
        auto future = client_->async_send_goal(goal, options);
    }
private:
    // 处理目标值相关响应结果
    void goal_response_callback(std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>>> goal_handle){
        if(goal_handle.get() == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "请求目标非法");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标值被接收");
        }
    }
    void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>> goal_handle, std::shared_ptr<const Nav::Feedback> feedback){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "剩余 %.2f m", feedback->distance);
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult &result){
        if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "成功响应最终结果：坐标(%.2f,%.2f)", result.result->turtle_x,result.result->turtle_y);
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

    rclcpp_action::Client<Nav>::SharedPtr client_;
};


int main(int argc, char ** argv)
{
    if(argc != 5)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "请输入 x y theta");
        return 1;
    }

    
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Exer05ActionClient>();
    client->send_goal(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
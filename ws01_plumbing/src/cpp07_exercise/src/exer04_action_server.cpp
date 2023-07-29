

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base_interfaces_demo/action/nav.hpp"
/**
 * 处理客户端发送的请求数据（目标点），控制乌龟向目标点运行，连续反馈剩余距离
 * 
 * 
 * 创建原生乌龟位姿订阅方，获取当前乌龟坐标
 * 创建速度指令发布方，控制乌龟运动
 * 创建动作服务端
 *      解析动作客户端提交的数据
 *      处理客户端取消请求操作  
 *      实现主逻辑（耗时操作），启动子线程
 *      子线程中，发布速度指令，产生连续反馈，并响应最终结果
*/
using std::placeholders::_1;
using std::placeholders::_2;
using base_interfaces_demo::action::Nav;
using geometry_msgs::msg::Twist;

class Exer04ActionServer: public rclcpp::Node{
public:
    Exer04ActionServer():
        Node("exer05_action_server_cpp"),
        x(0.0) ,
        y(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "创建动作服务端");
        // 创建一个订阅方,订阅乌龟位姿 /turtle1/pose
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&Exer04ActionServer::pose_cb, this, _1)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
/**
1. 
rclcpp_action::Server<ActionT>::SharedPtr 
create_server<ActionT, NodeT>(
    NodeT node, 
    const std::string &name, 
    rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
    rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
    rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
    const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
    rclcpp::CallbackGroup::SharedPtr group = nullptr)

2. 
rclcpp_action::Server<ActionT>::SharedPtr 
create_server<ActionT>(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface, 
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface, 
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface, 
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface, 
    const std::string &name, 
    rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
    rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
    rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
    const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
    rclcpp::CallbackGroup::SharedPtr group = nullptr)

*/
        action_server_ = rclcpp_action::create_server<Nav>(
            this, 
            "nav",
            std::bind(&Exer04ActionServer::handle_goal, this, _1, _2),
            std::bind(&Exer04ActionServer::handle_cancel, this, _1),
            std::bind(&Exer04ActionServer::handle_accepted, this, _1)
            );
    }
private:
    void pose_cb(const turtlesim::msg::Pose::SharedPtr pose){
        x = pose->x;
        y = pose->y;
    }
// GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Nav::Goal> goal){ // 请求目标处理
        (void)uuid;
        // (void)goal;
        // 取出目标中的 x y 坐标，判断是否超出取值范围
        if(goal->goal_x < 0 || goal->goal_x > 11.08 || goal->goal_y < 0 || goal->goal_y > 11.08)
        {
            RCLCPP_WARN(this->get_logger(), "目标点超出正常取值范围");
            return rclcpp_action::GoalResponse::REJECT;    
        }
        RCLCPP_INFO(this->get_logger(), "目标点合法");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
// CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){ // 取消请求处理
        (void)goal_handle;
        RCLCPP_WARN(this->get_logger(), "取消任务");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
// void (std::shared_ptr<ServerGoalHandle<ActionT>>)
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){ // 主逻辑处理
        // (void)goal_handle;
        std::thread(std::bind(&Exer04ActionServer::execute, this, goal_handle)).detach();
    }
// 子线程处理主要逻辑
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        // 1. 连续反馈
        RCLCPP_INFO(this->get_logger(), "主逻辑开始执行");
        rclcpp::Rate rate(1.0);
        // 最终结果
        auto result = std::make_shared<Nav::Result>();
        auto feedback = std::make_shared<Nav::Feedback>();
        Twist cmd_vel;
        while (true)
        {

            // 如果客户端发送了取消请求，需要特殊处理
            if(goal_handle->is_canceling())
            {
                // 设置取消后的最终结果
                goal_handle->canceled(result);
                return;
            }
            // 解析目标点坐标与原生乌龟实时坐标
            float goal_x = goal_handle->get_goal()->goal_x;
            float goal_y = goal_handle->get_goal()->goal_y;
            // 计算剩余距离，并发布
            float dis_x = goal_x - x;
            float dis_y = goal_y - y;
            float distance = std::sqrt(dis_x * dis_x + dis_y * dis_y);
            feedback->distance = distance;
            goal_handle->publish_feedback(feedback);
            // 根据剩余距离。计算速度指令并发布
            float scale = 0.5;
            float linear_x = scale * dis_x;
            float linear_y = scale * dis_y;
        // 2. 发布乌龟运动指令
            cmd_vel.linear.x = linear_x;
            cmd_vel.linear.y = linear_y;
            pub_->publish(cmd_vel);
            // 循环结束条件
            if(distance <= 0.05)
            {
                // 与目标点剩余距离小于0.05，结束导航
                RCLCPP_INFO(this->get_logger(), "乌龟已经导航至目标点");
                break;
            }
            rate.sleep();
        }
        // 3. 最终响应
        if(rclcpp::ok())
        {
            result->turtle_x = x;
            result->turtle_y = y;
            goal_handle->succeed(result);
        }
    }


    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp_action::Server<Nav>::SharedPtr action_server_;
    float x, y;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Exer04ActionServer>());
    rclcpp::shutdown();
    return 0;
}
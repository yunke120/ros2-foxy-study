#include "rclcpp/rclcpp.hpp"




// 构造函数创建客户端
// 连接服务端


#include "base_interfaces_demo/srv/distance.hpp"



using base_interfaces_demo::srv::Distance;
using namespace std::chrono_literals;
class Exer03Client: public rclcpp::Node{
public:
    Exer03Client():Node("exer03_client_cpp"){
        RCLCPP_INFO(this->get_logger(), "exer03_client_cpp");

        client_ = this->create_client<Distance>("distance");
    }

    bool connect_server(){
        while (client_->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Force Exit!!!");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Server Connect ...");
        }
        return true;
    }

    rclcpp::Client<Distance>::SharedFuture send_goal(float x, float y, float theta){
        // distance = Distance();
        auto request = std::make_shared<Distance::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;

        return client_->async_send_request(request);
    }
private: 
    rclcpp::Client<Distance>::SharedPtr client_;
};
int main(int argc, char ** argv)
{

    if(argc != 5) return 1;

    float goal_x =  atof(argv[1]);
    float goal_y =  atof(argv[2]);

    RCLCPP_INFO(rclcpp::get_logger("dd"), "(%.2f, %.2f)", goal_x, goal_y);


    rclcpp::init(argc, argv);
    auto node = std::make_shared<Exer03Client>();
    bool flag = node->connect_server();
    if(!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接失败");
        return 0;
    }
    auto future = node->send_goal(goal_x, goal_y, 0.0);
    // 判断响应结果状态
    if(rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "距离: %.2f", future.get()->distance);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Get Future Failed.");
    }
    
    rclcpp::shutdown();
    return 0;
}
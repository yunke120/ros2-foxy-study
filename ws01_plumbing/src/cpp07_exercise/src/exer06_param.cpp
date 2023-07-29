#include "rclcpp/rclcpp.hpp"

/*
    修改turtlesim_node背景色

*/
using namespace std::chrono_literals;
class Exer06Param : public rclcpp::Node
{
public:
    Exer06Param() : Node("exer06_param_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "参数客户端");

        client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/turtlesim");
    }

    bool connect_server()
    {

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "客户端强制推出");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中...");
            return false;
        }

        return true;
    }
    void update_param()
    {
        // 背景色递进修改
        // 1. 获取参数
        int red = client_->get_parameter<int>("background_r");
        bool dir = true;
        // 2. 编写循环，修改参数（100ms）
        rclcpp::Rate rate(10);
        while (rclcpp::ok())
        {
            if(dir)
                red += 5;
            else
                red -= 5;
            
            if(red <= 0)
            {
                red = 0;
                dir = true;
            }
            if(red >= 255)
            {
                red = 255;
                dir = false;
            }
            client_->set_parameters({rclcpp::Parameter("background_r", red)});
            rate.sleep();
        }
        
    }

private:
    rclcpp::SyncParametersClient::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Exer06Param>();
    if (!client->connect_server())
    {
        return 1;
    }
    client->update_param();
    // rclcpp::spin(std::make_shared<Exer06Param>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"

#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

class AddIntsClient: public rclcpp::Node{
public:
    AddIntsClient():Node("addints_client_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建客户端节点 CPP");

        // 1. 创建客户端
        /*
            模板：服务接口
            参数：话题名称
            返回值：服务对象指针
        */
        client_ = this->create_client<AddInts>("add_ints");

    }

    // 2。 连接服务器
    bool connect_server()
    {
        // 在制定超时时间内连接服务器，连接成功，返回true，反之，返回false.
        // client_->wait_for_service(1s);
        while(!client_->wait_for_service(2s)) // 以1s为超时时间，直到连接成功才退出循环
        {
            // 对 Ctrl + C作出特殊处理
            // 1. 如何判断ctrl+c按下   2. 如何处理
            if(!rclcpp::ok()) // 判断程序是否正确执行
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "强行终止客户端");
                return false;
            }
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接中...");
        }
        return true;
    }

    // 3. 发送请求
    // 参数是两个整形数据，返回值是响应数据
    rclcpp::Client<AddInts>::SharedFuture send_request(int num1, int num2)
    {
        /*
        rclcpp::Client<base_interfaces_demo::srv::AddInts>::SharedFuture 
        async_send_request(std::shared_ptr<base_interfaces_demo::srv::AddInts_Request> request)
        */
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};


int main(int argc, char ** argv)
{
    if(argc != 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整形数");
      return 1;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<AddIntsClient>();
    // 调用客户端连接服务器功能
    bool flag = client->connect_server();
    if(!flag)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "服务器连接失败，程序退出");
        return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接成功");

    // 调用请求提交汉书，接收并处理响应结果
    auto future = client->send_request(atoi(argv[1]), atoi(argv[2]));
    // 处理响应
    if(rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应成功， sum = %d", future.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "响应失败");
    }
    rclcpp::shutdown();
    return 0;
}
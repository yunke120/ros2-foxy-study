

#include "rclcpp/rclcpp.hpp"

/**
 * 创建参数客户端。
 * 查询或修改服务端参数
 * 
 * 1. 创建参数客户端对象，连接服务端
 * 2. 查询参数
 * 3. 参数修改
 * 
*/

/**
 *      服务通信不是通过服务话题关联吗？为什么参数参数客户端是通过服务端节点名称关联？？
 *          1. 参数服务端启动后，底层封装了多个服务通信的服务端
 *          2. 每个服务端的话题，都采用 /服务端节点名词/xxx
 *          3. 参数客户端创建后，也会封装多个服务通信的客户端
 *          4. 这些客户端与服务端相呼应，也要使用相同的话题  
 *          
*/
using namespace std::chrono_literals;

class MyParamClient: public rclcpp::Node{

public:
    MyParamClient():Node("my_param_client_node"){
        RCLCPP_INFO(this->get_logger(), "参数客户端创建 CPP");

        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
            this,                   // 当前对象依赖的节点
            "my_param_server_node"  // 远程服务端节点名称
        );
        
    }

    bool connect_server(){
        while(!param_client_->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {    
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中..."); 
        }
        RCLCPP_INFO(this->get_logger(), "服务连接成功"); 
        return true;
    }

    void get_param(){
        RCLCPP_INFO(this->get_logger(), "参数查询"); 
        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        double width = param_client_->get_parameter<double>("width");
        RCLCPP_INFO(this->get_logger(), "car_name: %s", car_name.c_str()); 
        RCLCPP_INFO(this->get_logger(), "width: %.2f", width); 

        auto params = param_client_->get_parameters({"car_name", "width", "wheels"});
        for (auto &&param : params) 
        {
            RCLCPP_INFO(this->get_logger(), "%s: %s", param.get_name().c_str(), param.value_to_string().c_str());  
        }
        
        RCLCPP_INFO(this->get_logger(), "包含car_name : %d", param_client_->has_parameter("car_name"));  
    }

    void update_param(){
        RCLCPP_INFO(this->get_logger(), "参数修改"); 
        param_client_->set_parameters({
            rclcpp::Parameter("car_name", "pig"),
            rclcpp::Parameter("width", 3.77),
            rclcpp::Parameter("length", 9.6
            ),
        });
    }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;

};




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<MyParamClient>();
    bool flag = client->connect_server();
    if(!flag)    return 0;
    client->get_param();
    client->update_param();
    client->get_param();

    rclcpp::shutdown();
    return 0;
}

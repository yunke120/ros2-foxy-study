

#include "rclcpp/rclcpp.hpp"

class MyParamServer: public rclcpp::Node{
public:
    // 如果允许参数删除，需要通过NodeOption来声明
    MyParamServer():Node("my_param_server_node", rclcpp::NodeOptions().allow_undeclared_parameters(true)){
        RCLCPP_INFO(this->get_logger(), "参数服务端创建 CPP");
        
        // 增
        
        // 查

        // 改

        // 删

    }

    void declare_param(){
        RCLCPP_INFO(this->get_logger(), "------------- 增 -------------");

        this->declare_parameter("car_name", "tiger");
        this->declare_parameter("width", 1.66);
        this->declare_parameter("wheels", 5);

        this->set_parameter(rclcpp::Parameter("height", 1.88));
    }

    void get_param(){
        RCLCPP_INFO(this->get_logger(), "------------- 查 -------------");
        
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(), "key = %s, val = %s", car.get_name().c_str(), car.as_string().c_str()); 

        auto params = this->get_parameters({"car_name", "wheels", "width"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "(%s = %s)", param.get_name().c_str(), param.value_to_string().c_str()); 
        }
        
        RCLCPP_INFO(this->get_logger(), "是否包含car_name   : %d", this->has_parameter("car_name")); 
        RCLCPP_INFO(this->get_logger(), "是否包含height     : %d", this->has_parameter("height")); 

    }

    void update_param(){
        RCLCPP_INFO(this->get_logger(), "------------- 改 -------------");

        this->set_parameter(rclcpp::Parameter("width", 1.75));
    }

    void del_param(){
        RCLCPP_INFO(this->get_logger(), "------------- 删 -------------");
        this->undeclare_parameter("car_name");
        RCLCPP_INFO(this->get_logger(), "是否包含car_name   : %d", this->has_parameter("car_name")); 
    }
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyParamServer>();
    node->declare_param();
    node->get_param();
    // node->update_param();
    // node->get_param();
    // node->del_param();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
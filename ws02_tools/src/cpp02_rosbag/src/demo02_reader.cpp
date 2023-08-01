#include "rclcpp/rclcpp.hpp"


/*
    读取bag文件，将数据输出在终端

    创建一个回放1对象
    设置被读取的文件
    读消息
    关闭文件
*/

class SimpleBagPlay: public rclcpp::Node{
public:
    SimpleBagPlay():Node("simplebagplay_cpp"){
        RCLCPP_INFO(this->get_logger(), "消息回放对象创建");
        
    }
private:

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleBagPlay>());
    rclcpp::shutdown();
    return 0;
}
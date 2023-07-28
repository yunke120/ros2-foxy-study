
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

// 创建发布方

using base_interfaces_demo::msg::Student;

class ListenerStu: public rclcpp::Node{
public:
    ListenerStu():Node("listener_stu"){
        subscription_ = this->create_subscription<Student>("chatter_stu", 10, std::bind(&ListenerStu::on_cb, this, std::placeholders::_1));
    }
private:
    void on_cb(const Student::SharedPtr stu)
    {
        RCLCPP_INFO(this->get_logger(), "订阅的消息: (%s,%d,%f)", stu->name.c_str(), stu->age, stu->height);
    }
    
    rclcpp::Subscription<Student>::SharedPtr subscription_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerStu>());
    rclcpp::shutdown();
    return 0;
}
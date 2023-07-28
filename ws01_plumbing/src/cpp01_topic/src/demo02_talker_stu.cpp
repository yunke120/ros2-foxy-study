
#include "rclcpp/rclcpp.hpp"

#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;

class TalkerStu: public rclcpp::Node{
public:
    TalkerStu():Node("talker_stu"){

        // 创建发布方
        publisher_ = this->create_publisher<Student>("chatter_stu", 10);
        // 创建定时器
        timer_ = this->create_wall_timer(500ms, std::bind(&TalkerStu::on_timer, this));

    }

private:
    void on_timer(){
        auto stu = Student();
        stu.name = "fujun";
        stu.age = 26;
        stu.height = 175.0;
        publisher_->publish(stu);
        RCLCPP_INFO(this->get_logger(), "发布的消息: (%s,%d,%f)", stu.name.c_str(), stu.age, stu.height);
    }

    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerStu>());
    rclcpp::shutdown();
    return 0;
}


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Listener : public rclcpp::Node{
public:
  Listener():Node("listerer_node_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "订阅节点创建！");
    subscription_ = this->create_subscription<std_msgs::msg::String>("chatter", 10,  std::bind(&Listener::on_cb ,this,std::placeholders::_1));
    
  }
private:

  void on_cb(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "订阅消息: %s", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Listener>());

  rclcpp::shutdown();
  return 0;
}


// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// using std::placeholders::_1;

// class Listener : public rclcpp::Node {
// public:
//   Listener() : Node("listener_node_cpp") {
//     RCLCPP_INFO(this->get_logger(), "订阅节点创建！");
//     subscription_ = this->create_subscription<std_msgs::msg::String>(
//         "chatter", rclcpp::QoS(rclcpp::KeepAll()), std::bind(&Listener::on_cb, this, std::placeholders::_1));
//   }

// private:
//   void on_cb(const std_msgs::msg::String::SharedPtr msg) {
//     RCLCPP_INFO(this->get_logger(), "订阅消息: %s", msg->data.c_str());
//   }

//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Listener>());
//   rclcpp::shutdown();
//   return 0;
// }
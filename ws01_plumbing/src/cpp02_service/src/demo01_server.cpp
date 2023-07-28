
#include "rclcpp/rclcpp.hpp"

#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using std::placeholders::_1;
using std::placeholders::_2;
class AddIntsServer: public rclcpp::Node{
public:
    AddIntsServer():Node("addints_server_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建服务端节点 CPP");

        server_ = this->create_service<AddInts>("add_ints", std::bind(&AddIntsServer::add, this, _1, _2));
    }

private:
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res)
    {
        res->sum = req->num1 + req->num2;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", req->num1, req->num2, res->sum);
    }
  rclcpp::Service<AddInts>::SharedPtr server_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddIntsServer>());
    rclcpp::shutdown();
    return 0;
}


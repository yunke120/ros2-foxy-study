#include "rclcpp/rclcpp.hpp"

#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

// 解析客户端提交的目标点坐标
// 获取原声乌龟坐标
// 计算二者距离并响应客户端


// 创建一个订阅方,订阅乌龟位姿 /turtle1/pose
// 创建一个服务端
// 回调函数需要解析客户端数据并响应

using base_interfaces_demo::srv::Distance;
using std::placeholders::_1;
using std::placeholders::_2;
class Exer02Server: public rclcpp::Node{

public:
    Exer02Server():
        Node("exer02_server_cpp") ,
        x(0.0) ,
        y(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "exer02_server_cpp");
        // 创建一个订阅方,订阅乌龟位姿 /turtle1/pose
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&Exer02Server::pose_cb, this, std::placeholders::_1)
        );
        // 创建一个服务端
        server_ = this->create_service<Distance>(
            "distance",
            std::bind(&Exer02Server::distance_cb, this, _1, _2)
        );
        // 回调函数需要解析客户端数据并响应
   
   
    }
private:
    void pose_cb(const turtlesim::msg::Pose::SharedPtr pose){
        x = pose->x;
        y = pose->y;
    }

    void distance_cb(const Distance::Request::SharedPtr req, Distance::Response::SharedPtr res){
        // 解析出目标点坐标
        float goal_x = req->x;
        float goal_y = req->y;
        // 计算距离
        float dis_x = goal_x - x; 
        float dis_y = goal_y - y; 
        float dis = std::sqrt(dis_x * dis_x + dis_y * dis_y);
        // 设置进响应
        res->distance = dis;

        RCLCPP_INFO(this->get_logger(), "(%.2f, %.2f)<->(%.2f, %.2f): %.2f", x, y, goal_x, goal_y, dis);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Service<Distance>::SharedPtr server_;
    float x, y;
};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Exer02Server>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

class TimeNode: public rclcpp::Node{
public:
    TimeNode():Node("time_node_cpp"){
        // demo_rate();
        // demo_time();
        // demo_duration();
        demo_opt();
    }


private:
    void demo_rate(){
        // 创建rate对象
        rclcpp::Rate rate1(1s);
        rclcpp::Rate rate2(1.0);
        //调用sleep
        while (rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "-----------");
            // rate1.sleep();
            rate2.sleep();
        }
        
        
    }

    void demo_time(){
        // 创建time对象
        rclcpp::Time time1(500000000L);
        rclcpp::Time time2(2, 500000000L);
        rclcpp::Time right_now = this->get_clock()->now();
        rclcpp::Time right_now2 = this->now();


        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", time1.seconds(), time1.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", time2.seconds(), time2.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", right_now2.seconds(), right_now2.nanoseconds());

    }

    void demo_duration(){
      rclcpp::Duration du1(1s);
      rclcpp::Duration du2(1, 500000000);

      RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", du1.seconds(), du1.nanoseconds());
      RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", du2.seconds(), du2.nanoseconds());
    }

    void demo_opt(){
      // 演示运算法的使用
      rclcpp::Time t1(10, 0);
      rclcpp::Time t2(30, 0);

      rclcpp::Duration d1(10, 0);
      rclcpp::Duration d2(30, 0);

      RCLCPP_INFO(this->get_logger(), "t1 >= t2 ? %d", t1 >= t2);
      RCLCPP_INFO(this->get_logger(), "t1 < t2 ? %d", t1 < t2);
      rclcpp::Duration d3 = t2 - t1;
      rclcpp::Time t3 = t1 + d1;
      rclcpp::Time t4 = t1 - d1;

      RCLCPP_INFO(this->get_logger(), "d3 = %2f", d3.seconds());  // 20
      RCLCPP_INFO(this->get_logger(), "t3 = %2f", t3.seconds());  // 20
      RCLCPP_INFO(this->get_logger(), "t4 = %2f", t4.seconds());  // 0

      RCLCPP_INFO(this->get_logger(), "d1 >= d2 ? %d", d1 >= d2);
      RCLCPP_INFO(this->get_logger(), "d1 < d2 ? %d", d1 < d2);
      rclcpp::Duration d4 = d1 * 3;
      rclcpp::Duration d5 = d2 - d1;
      rclcpp::Duration d6 = d1 + d2;
      RCLCPP_INFO(this->get_logger(), "d4 = %2f", d4.seconds());  // 30
      RCLCPP_INFO(this->get_logger(), "d5 = %2f", d5.seconds());  // 30
      RCLCPP_INFO(this->get_logger(), "d6 = %2f", d6.seconds());  // 30

    }
};




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimeNode>());
    rclcpp::shutdown();
    return 0;
}
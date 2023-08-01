#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using std::placeholders::_1;

/*
  ROS2 Foxy官方代码

  RUN：
    启动chatter话题 ： ros2 run demo_nodes_cpp talker
    启动writer:       ros2 run cpp02_rosbag demo01_writer
    记录一段时间后关闭以上两个节点

    运行 ros2 run demo_nodes_cpp listener
    运行 ros2 bag play my_bag
*/


class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    const rosbag2_cpp::StorageOptions storage_options({"my_bag", "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"chatter",
       "std_msgs/msg/String",
       rmw_get_serialization_format(),
       ""});

    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t *msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(get_logger(),
            "Failed to destroy serialized message %s", rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data = msg->release_rcl_serialized_message();

    bag_message->topic_name = "chatter";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
  }

  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
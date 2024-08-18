#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "M_VSLAM/frontend.h"
class Sub : public rclcpp::Node
{
public:
    Sub()
    : Node("slam"){
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&Sub::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const{
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sub>());
    rclcpp::shutdown();
    return 0;
}
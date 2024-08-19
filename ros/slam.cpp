#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>
#include "M_VSLAM/frontend.h"
class Sub : public rclcpp::Node
{
public:
    Sub()
    : Node("slam"){
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&Sub::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&Sub::timer_callback, this)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const{
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void timer_callback(){
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sub>());
    rclcpp::shutdown();
    return 0;
}
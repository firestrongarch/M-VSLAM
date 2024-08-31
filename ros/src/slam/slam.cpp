#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "slam_msg/msg/camera.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>
#include "M_VSLAM/frontend.h"
class Sub : public rclcpp::Node
{
public:
    Sub()
    : Node("slam"){
        subscription_ = this->create_subscription<slam_msg::msg::Camera>(
            "image_left", 
            10, 
            std::bind(&Sub::topic_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&Sub::timer_callback, this)
        );
    }

private:
    void topic_callback(const slam_msg::msg::Camera::SharedPtr msg) const{
        auto lImagePtr = cv_bridge::toCvCopy(msg->image0, msg->image0.encoding);
        auto rImagePtr = cv_bridge::toCvCopy(msg->image1, msg->image1.encoding);
        Frontend slam;
        slam.RunBinocular(lImagePtr->image, rImagePtr->image, 1);
    }

    rclcpp::Subscription<slam_msg::msg::Camera>::SharedPtr subscription_;
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
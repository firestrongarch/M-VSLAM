#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("slam");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
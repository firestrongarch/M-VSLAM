#include <rclcpp/rclcpp.hpp>

int main()
{
    rclcpp::init(0, nullptr);
    rclcpp::shutdown();
    return 0;
}
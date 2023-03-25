#include <cstdio>
#include "../include/realsense2_health_checker.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<realsenseHealthChecker::RealsenseHealthChecker>("realsense_health_checker");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

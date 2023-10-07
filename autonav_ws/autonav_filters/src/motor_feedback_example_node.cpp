#include "rclcpp.hpp"
#include "autonav_filters/motor_feedback_example.hpp"

int main (int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPublisher>());
    rclcpp::shutdown();
    return 0;
}
#include "autonav_filters/motor_feedback_example.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "autonav_messages/msg/motor_feedback.hpp"

using namespace std::chrono_literals;

void PositionPublisher::timer_callback() {
    autonav_messages::msg::MotorFeedback message = autonav_messages::msg::MotorFeedback();
    message.delta_x = 10;
    message.delta_y = 10;
    message.delta_theta = 360;
    
    RCLCPP_INFO(this->get_logger(), "Publishing: delta_x: %f, delta_y: %f, delta_theta: %f"
                                ,message.delta_x, message.delta_y, message.delta_theta);
    publisher_->publish(message);
}

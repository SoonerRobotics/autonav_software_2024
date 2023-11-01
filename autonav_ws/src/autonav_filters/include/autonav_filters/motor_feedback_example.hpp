#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_feedback.hpp"

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node {
    public:
        PositionPublisher() : Node("GPS_feedback_publisher"), count_(0)
        {
            publisher_ = this->create_publisher<autonav_msgs::msg::MotorFeedback>("topic", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&PositionPublisher::timer_callback, this));
        }

    private:
        void timer_callback();
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<autonav_msgs::msg::MotorFeedback>::SharedPtr publisher_;
        size_t count_;
};
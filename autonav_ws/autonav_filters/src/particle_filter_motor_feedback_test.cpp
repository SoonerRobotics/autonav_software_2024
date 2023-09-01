#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autonav_messages/msg/motor_feedback.hpp"

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node {
    public:
    PositionPublisher() : Node("GPS_feedback_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<autonav_messages::msg::MotorFeedback>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&PositionPublisher::timer_callback, this));
    }

    private:
        void timer_callback() {
            autonav_messages::msg::MotorFeedback message = autonav_messages::msg::MotorFeedback();
            message.delta_x = 10;
            message.delta_y = 10;
            message.delta_theta = 360;
            
            RCLCPP_INFO(this->get_logger(), "Publishing: delta_x: %f, delta_y: %f, delta_theta: %f"
                                        ,message.delta_x, message.delta_y, message.delta_theta);
            publisher_->publish(message);
        }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<autonav_messages::msg::MotorFeedback>::SharedPtr publisher_;
    size_t count_;
};

int main (int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPublisher>());
    rclcpp::shutdown();
    return 0;

}
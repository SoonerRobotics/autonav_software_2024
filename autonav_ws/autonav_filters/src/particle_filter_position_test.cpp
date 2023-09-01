#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autonav_messages/msg/position.hpp"

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node {
    public:
    PositionPublisher() : Node("position_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<autonav_messages::msg::Position>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&PositionPublisher::timer_callback, this));
    }

    private:
        void timer_callback() {
            autonav_messages::msg::Position message = autonav_messages::msg::Position();
            message.x = 1;
            message.y = 5;
            message.theta = 314;
            message.latitude = 1000;
            message.longitude = 5000;
            
            RCLCPP_INFO(this->get_logger(), "Publishing: x: %f, y: %f, theta: %f, latitude: %f, longitude: %f"
                                        ,message.x, message.y, message.theta, message.latitude, message.longitude);
            publisher_->publish(message);
        }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<autonav_messages::msg::Position>::SharedPtr publisher_;
    size_t count_;
};

int main (int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPublisher>());
    rclcpp::shutdown();
    return 0;

}
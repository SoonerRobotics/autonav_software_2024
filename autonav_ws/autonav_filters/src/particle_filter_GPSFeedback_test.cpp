#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autonav_messages/msg/gps_feedback.hpp"

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node {
    public:
    PositionPublisher() : Node("GPS_feedback_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<autonav_messages::msg::GPSFeedback>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&PositionPublisher::timer_callback, this));
    }

    private:
        void timer_callback() {
            autonav_messages::msg::GPSFeedback message = autonav_messages::msg::GPSFeedback();
            message.latitude = 1000;
            message.longitude = 5000;
            message.altitude = 2000;
            message.gps_fix = 10;
            message.is_locked = false;
            message.satellites = 123;
            
            RCLCPP_INFO(this->get_logger(), "Publishing: lat: %f, long: %f, alt: %f, gps_fix: %f, is_locked: %d, satellites: %f"
                                        ,message.latitude, message.longitude, message.altitude, message.gps_fix, message.gps_fix, message.satellites);
            publisher_->publish(message);
        }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<autonav_messages::msg::GPSFeedback>::SharedPtr publisher_;
    size_t count_;
};

int main (int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPublisher>());
    rclcpp::shutdown();
    return 0;

}
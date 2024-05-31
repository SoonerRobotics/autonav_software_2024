#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "autonav_messages/msg/motor_feedback.hpp"
#include "autonav_messages/msg/gps_feedback.hpp"
#include "autonav_messages/msg/position.hpp"

#include "autonav_filters/particle_filter.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

class FiltersNode : public rclcpp::Node {
    public:
        FiltersNode() : Node("autonav_filters"), count_(0)
        {
            // subscriptions
            gps_subscription = this->create_subscription<autonav_messages::msg::GPSFeedback>("/autonav_GPS", 
            20, std::bind(&FiltersNode::on_GPS_received, this, std::placeholders::_1));
            motor_subscription = this->create_subscription<autonav_messages::msg::MotorFeedback>("/autonav_MotorFeedback", 
            20, std::bind(&FiltersNode::on_MotorFeedback_received, this, std::placeholders::_1));

            // publishers
            positionPublisher = this->create_publisher<autonav_messages::msg::Position>("/autonav/position", 20);
        }

    private:
        //particle filter
        float latitudeLength = 111086.2;
        float longitudeLength = 81978.2;
        ParticleFilter particle_filter{latitudeLength, longitudeLength};
        autonav_messages::msg::GPSFeedback first_gps;
        autonav_messages::msg::GPSFeedback last_gps;
        bool first_gps_received = false;
        
        void on_GPS_received(const autonav_messages::msg::GPSFeedback gps_message);

        void on_MotorFeedback_received(const autonav_messages::msg::MotorFeedback motorFeedback_message);

        rclcpp::Subscription<autonav_messages::msg::GPSFeedback>::SharedPtr gps_subscription;
        rclcpp::Subscription<autonav_messages::msg::MotorFeedback>::SharedPtr motor_subscription;
        rclcpp::Publisher<autonav_messages::msg::Position>::SharedPtr positionPublisher;
        size_t count_;
};
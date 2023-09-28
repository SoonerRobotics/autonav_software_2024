#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "autonav_messages/msg/motor_feedback.hpp"
#include "autonav_messages/msg/gps_feedback.hpp"
#include "autonav_messages/msg/position.hpp"

#include "autonav_filters/src/particle_filter.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

class FiltersNode : public rclcpp::Node {
    public:
        FiltersNode() : Node("autonav_filters"), count_(0)
        {
            // subscriptions
            gps_subscription = this->create_subscription<autonav_messages::msg::GPSFeedback>("/autonav_GPS", 
            20, std::bind(&FiltersNode::on_GPS_received));
            motor_subscription = this->create_subscription<autonav_messages::msg::MotorFeedback>("/autonav_MotorFeedback", 
            20, std::bind(&FiltersNode::on_MotorFeedback_received));

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
        
        void on_GPS_received(const autonav_messages::msg::GPSFeedback gps_message) {
            if (gps_message.gps_fix == 0 &&  gps_message.is_locked == false) {
                return;
            }
            if (this->first_gps_received = false) {
                this->first_gps = gps_message;
                this->first_gps_received = true;
            }

            this->last_gps = gps_message;

            this->particle_filter.gps(gps_message);

        }
        void on_MotorFeedback_received(const autonav_messages::msg::MotorFeedback motorFeedback_message) {
            std::vector<float> averages;
            averages = this->particle_filter.feedback(motorFeedback_message);

            if (averages.size() < 3) {
                return;
            }

            autonav_messages::msg::Position position;
            position.x = averages[0];
            position.y = averages[1];
            position.theta = (-1 * M_PI * 2 + averages[2]);

            if (this->first_gps_received = true) {
                float gps_x = this->first_gps.latitude + position.x / this->latitudeLength;
                float gps_y = this->first_gps.longitude - position.y / this->longitudeLength;

                position.latitude = gps_x;
                position.longitude = gps_y;
            }

            positionPublisher->publish(position);
        }

        rclcpp::Subscription<autonav_messages::msg::GPSFeedback>::SharedPtr gps_subscription;
        rclcpp::Subscription<autonav_messages::msg::MotorFeedback>::SharedPtr motor_subscription;
        rclcpp::Publisher<autonav_messages::msg::Position>::SharedPtr positionPublisher;
        size_t count_;
};
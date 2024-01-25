#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "scr/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/imu_data.hpp"

#include "autonav_filters/particle_filter.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

class FiltersNode : public SCR::Node {
    public:
        FiltersNode() : SCR::Node("autonav_filters"), count_(0)
        {
            // subscriptions
            gps_subscription = this->create_subscription<autonav_msgs::msg::GPSFeedback>("/autonav_GPS", 
            20, std::bind(&FiltersNode::on_GPS_received, this, std::placeholders::_1));
            motor_subscription = this->create_subscription<autonav_msgs::msg::MotorFeedback>("/autonav_MotorFeedback", 
            20, std::bind(&FiltersNode::on_MotorFeedback_received, this, std::placeholders::_1));

            // publishers
            positionPublisher = this->create_publisher<autonav_msgs::msg::Position>("/autonav/position", 20);
            
            this->on_reset();
        }

    private:
        //particle filter
        double latitudeLength = 111086.2;
        double longitudeLength = 81978.2;
        double degreeOffset = 107.0;
        ParticleFilter particle_filter{latitudeLength, longitudeLength};
        autonav_msgs::msg::GPSFeedback first_gps;
        autonav_msgs::msg::GPSFeedback last_gps;
        bool first_gps_received = false;
        bool first_imu_received = false;
        autonav_msgs::msg::IMUData last_IMU_received;

        void on_IMU_received(const autonav_msgs::msg::IMUData IMU_msg);

        double get_real_heading(float heading);

        void on_reset();

        void system_state_transition();
        
        void on_GPS_received(const autonav_msgs::msg::GPSFeedback gps_message);

        void on_MotorFeedback_received(const autonav_msgs::msg::MotorFeedback motorFeedback_message);

        rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr gps_subscription;
        rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr motor_subscription;
        rclcpp::Publisher<autonav_msgs::msg::Position>::SharedPtr positionPublisher;
        size_t count_;
};
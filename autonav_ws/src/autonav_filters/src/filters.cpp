#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "autonav_filters/filters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/imu_data.hpp"
#include "scr/states.hpp"
#include "scr_msgs/msg/system_state.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

//particle filter
double latitudeLength = 111086.2;
double longitudeLength = 81978.2;
ParticleFilter particle_filter{latitudeLength, longitudeLength};
autonav_msgs::msg::GPSFeedback first_gps;
autonav_msgs::msg::GPSFeedback last_gps;
bool first_gps_received = false;

void FiltersNode::on_IMU_received(autonav_msgs::msg::IMUData IMU_msg) {
    this->last_IMU_received = IMU_msg;
}

double FiltersNode::get_real_heading(float heading) {
    if (heading < 0) {
        heading = 360 + -heading;
    }

    heading += this->degreeOffset;
    return heading;
}

void FiltersNode::on_reset() {
}

void FiltersNode::on_GPS_received(const autonav_msgs::msg::GPSFeedback gps_message) {
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
void FiltersNode::on_MotorFeedback_received(const autonav_msgs::msg::MotorFeedback motorFeedback_message) {
    std::vector<double> averages;
    averages = this->particle_filter.feedback(motorFeedback_message);

    if (averages.size() < 3) {
        return;
    }

    autonav_msgs::msg::Position position;
    position.x = averages[0];
    position.y = averages[1];
    position.theta = (-1 * M_PI * 2 + averages[2]);

    if (this->first_gps_received = true) {
        double gps_x = this->first_gps.latitude + position.x / this->latitudeLength;
        double gps_y = this->first_gps.longitude - position.y / this->longitudeLength;

        position.latitude = gps_x;
        position.longitude = gps_y;
    }

    positionPublisher->publish(position);
}

rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr gps_subscription;
rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr motor_subscription;
rclcpp::Publisher<autonav_msgs::msg::Position>::SharedPtr positionPublisher;
size_t count_;
#pragma once

#include "autonav_filters/filters.hpp"
#include <string>
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/imu_data.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

struct ParticleFilterConfig {
    int num_particles;
    double latitudeLength;
    double longitudeLength;
    double gps_noise;
    double odom_noise_x;
    double odom_noise_y;
    double odom_noise_theta;
    
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ParticleFilterConfig, num_particles, latitudeLength, longitudeLength, gps_noise, odom_noise_x, odom_noise_y, odom_noise_theta);
};

ParticleFilterConfig config;

//particle filter
void FiltersNode::init() {
    //double latitudeLength = 111086.2;
    //double longitudeLength = 81978.2;
    this->latitudeLength = config.latitudeLength;
    this->longitudeLength = config.longitudeLength;
    //ParticleFilter particle_filter{config.num_particles, config.latitudeLength, config.longitudeLength, config.gps_noise, config.odom_noise_x, config.odom_noise_y, config.odom_noise_theta};
    RCLCPP_INFO(this->get_logger(), "config.num_particles %d", config.num_particles);
    this->particle_filter = particle_filter;
    this->particle_filter.init_particles();
    //this->particle_filter.printParticles();
    autonav_msgs::msg::GPSFeedback first_gps;
    autonav_msgs::msg::GPSFeedback last_gps;
    bool first_gps_received = false;

    this->on_reset();
};

void FiltersNode::system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) {
    
};

// void FiltersNode::system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) {
//     if ((old.state != SCR::SystemState::AUTONOMOUS) && (updated.state == SCR::SystemState::AUTONOMOUS)) {
//         this->on_reset();
//     }

//     if ((old.mobility == false) and updated.mobility == true) {
//         this->on_reset();
//     }
// }

void FiltersNode::on_reset() {
    // if you're going to seed the pf with the IMU it would go here
    this->particle_filter.init_particles();
}

void FiltersNode::config_updated(json newConfig) {
        RCLCPP_INFO(this->get_logger(), "config updated");
        config = newConfig.template get<ParticleFilterConfig>();
        ParticleFilter particle_filter{config.num_particles, config.latitudeLength, config.longitudeLength, config.gps_noise, config.odom_noise_x, config.odom_noise_y, config.odom_noise_theta};
        this->particle_filter = particle_filter;
        this->particle_filter.init_particles();
        //this->particle_filter.printParticles();
}

json FiltersNode::get_default_config() {
        ParticleFilterConfig newConfig;
        newConfig.latitudeLength = 110944.12;
        newConfig.longitudeLength = 91071.17;
        newConfig.num_particles = 750; 
        newConfig.gps_noise = 0.8;
        newConfig.odom_noise_x = 0.05;
        newConfig.odom_noise_y = 0.05;
        newConfig.odom_noise_theta = 0.01;

        return newConfig;
}

void FiltersNode::on_GPS_received(autonav_msgs::msg::GPSFeedback gps_message) {
    if (gps_message.gps_fix == 0 &&  gps_message.is_locked == false) {
        return;
    }
    if (this->first_gps_received == false) {
        this->first_gps = gps_message;
        this->first_gps_received = true;
    }
        
    this->last_gps = gps_message;
    this->last_gps_assigned = true;

    this->particle_filter.gps(gps_message);
    
}
void FiltersNode::on_MotorFeedback_received(autonav_msgs::msg::MotorFeedback motorFeedback_message) {
    // RCLCPP_INFO(this->get_logger(), "received motorfeedback %f", 5);
    std::vector<double> averages;
    averages = this->particle_filter.feedback(motorFeedback_message);

    if (averages.size() < 3) {
        return;
    }

    autonav_msgs::msg::Position position;
    position.x = averages[0];
    position.y = averages[1];
    //position.theta = (-1 * M_PI * 2 + averages[2]) * 1;
    position.theta = averages[2];
    if (this->first_gps_received == true) {
        double gps_x = this->first_gps.latitude + position.x / this->latitudeLength;
        double gps_y = this->first_gps.longitude - position.y / this->longitudeLength;

        position.latitude = gps_x;
        position.longitude = gps_y;
    }

    if (this->last_gps_assigned = true) {
        //position.latitude = this->last_gps.latitude;
        //position.longitude = this->last_gps.longitude;
    }

    positionPublisher->publish(position);
}

rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr gps_subscription;
rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr motor_subscription;
rclcpp::Publisher<autonav_msgs::msg::Position>::SharedPtr positionPublisher;
size_t count_;

int main (int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<FiltersNode>());
    rclcpp::shutdown();
    return 0;
}
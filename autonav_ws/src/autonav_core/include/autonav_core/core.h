#pragma once
#include "rclcpp/rclcpp.hpp"
#include "autonav_core/performance_timer.hpp"
#include "autonav_core/device_state.hpp"
#include "autonav_core/robot_state.hpp"
#include "autonav_msgs/msg/robot_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"


namespace SCR
{
    class AutonavNode : public rclcpp::Node
    {
        public:
            AutonavNode(std::string node_name);
            ~AutonavNode();

        private:
            SCR::PerformanceTimer performance_timer = SCR::PerformanceTimer("default_timer");
            SCR::RobotState robot_state;
            SCR::DeviceState device_state;

            rclcpp::Subscription<autonav_msgs::msg::RobotState>::SharedPtr robot_state_subscription;
            rclcpp::Subscription<autonav_msgs::msg::DeviceState>::SharedPtr device_state_subscription;

            rclcpp::Publisher<autonav_msgs::msg::RobotState>::SharedPtr robot_state_publisher;

            void DeviceStateUpdate(autonav_msgs::msg::DeviceState::SharedPtr msg);
            void RobotStateUpdate(autonav_msgs::msg::RobotState::SharedPtr msg);
    };
}
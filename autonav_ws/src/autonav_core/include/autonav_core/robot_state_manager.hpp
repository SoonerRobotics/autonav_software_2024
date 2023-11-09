#pragma once
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/robot_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_core/device_state.hpp"
#include "autonav_core/robot_state.hpp"

namespace SCR {
    class RobotStateManagerNode : public rclcpp::Node {

        public:
            RobotStateManagerNode() : Node("robot_state_manager_node") {}
            ~RobotStateManagerNode();


        private:
            size_t count_;

            // device state management
            std::vector<Device> devices;

            Device get_device_by_name(std::string name);
            
            // robot state management
            RobotState robot_state;

            // subscriptions
            rclcpp::Subscription<autonav_msgs::msg::RobotState>::SharedPtr robot_state_subscription;
            rclcpp::Subscription<autonav_msgs::msg::DeviceState>::SharedPtr device_state_subscription;

            void robot_state_update(autonav_msgs::msg::RobotState::SharedPtr robot_state_msg);
            void device_state_update(autonav_msgs::msg::DeviceState::SharedPtr device_state_msg);
            
            // publishers
            rclcpp::Publisher<autonav_msgs::msg::RobotState>::SharedPtr robot_state_publisher;
            rclcpp::Publisher<autonav_msgs::msg::DeviceState>::SharedPtr device_state_publisher;
   };

}
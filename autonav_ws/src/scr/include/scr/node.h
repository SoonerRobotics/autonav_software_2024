#pragma once

#include "rclcpp/rclcpp.hpp"
#include <states.h>
#include <string.h>

namespace SCR
{
    class Node : public rclcpp::Node
    {
    public:
        Node(std::string node_name);
        ~Node();

    private:
        void systemModeCallback(const SCR::SystemMode::SharedPtr msg);
        void systemStateCallback(const SCR::SystemState::SharedPtr msg);
        void deviceStateCallback(const SCR::DeviceState::SharedPtr msg);
        void systemModeClientCallback(const SCR::SystemMode::Response::SharedPtr msg);
        void systemStateClientCallback(const SCR::SystemState::Response::SharedPtr msg);
        void deviceStateClientCallback(const SCR::DeviceState::Response::SharedPtr msg);

    protected:
        SCR::SystemMode system_mode = SCR::SystemMode::COMPETITION;
        SCR::SystemState system_state = SCR::SystemState::DISABLED;
        SCR::DeviceState device_state = SCR::DeviceState::OFF;
        std::string identifier;

    private:
        /// @brief Callback for system mode
        rclcpp::Subscription<SCR::SystemMode>::SharedPtr system_mode_sub;

        /// @brief Callback for system state
        rclcpp::Subscription<SCR::SystemState>::SharedPtr system_state_sub;
        
        /// @brief Callback for device state
        rclcpp::Subscription<SCR::DeviceState>::SharedPtr device_state_sub;

        /// @brief Client for system mode. Allows us to change the system mode
        rclcpp::Client<SCR::SystemMode>::SharedPtr system_mode_client;

        /// @brief Client for system state. Allows us to change the system state
        rclcpp::Client<SCR::SystemState>::SharedPtr system_state_client;

        /// @brief Client for device state. Allows us to change our device state
        rclcpp::Client<SCR::DeviceState>::SharedPtr device_state_client;
    };
}
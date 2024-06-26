#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "scr_msgs/srv/update_config.hpp"
#include "scr_msgs/srv/update_device_state.hpp"
#include "scr_msgs/srv/update_system_state.hpp"
#include "scr_msgs/msg/config_updated.hpp"
#include "scr_msgs/msg/device_state.hpp"
#include "scr_msgs/msg/system_state.hpp"
#include "scr_msgs/msg/log.hpp"
#include "states.hpp"
#include "structs.hpp"
#include "constants.hpp"
#include <string.h>

#include "json.hpp"
using json = nlohmann::json;

namespace SCR
{
    struct NodeSubscriptions
    {
        rclcpp::Subscription<scr_msgs::msg::SystemState>::SharedPtr system_state;
        rclcpp::Subscription<scr_msgs::msg::DeviceState>::SharedPtr device_state;
        rclcpp::Subscription<scr_msgs::msg::ConfigUpdated>::SharedPtr config_updated;
    };

    struct NodePublishers
    {
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr performance_track;
        rclcpp::Publisher<scr_msgs::msg::Log>::SharedPtr logging;
    };

    struct NodeClients
    {
        rclcpp::Client<scr_msgs::srv::UpdateSystemState>::SharedPtr system_state;
        rclcpp::Client<scr_msgs::srv::UpdateDeviceState>::SharedPtr device_state;
        rclcpp::Client<scr_msgs::srv::UpdateConfig>::SharedPtr config_update;
    };

    struct CallbackGroups
    {
        rclcpp::CallbackGroup::SharedPtr device_state;
        rclcpp::CallbackGroup::SharedPtr system_state;
        rclcpp::CallbackGroup::SharedPtr config_updated;
    };

    class Node : public rclcpp::Node
    {
    public:
        Node(std::string node_name);
        ~Node();

        /// @brief Set the system mode
        /// @param mode The new system mode
        void set_system_mode(SCR::SystemMode mode);

        /// @brief Set the system state
        /// @param state The new system state
        void set_system_state(SCR::SystemState state);

        /// @brief Set the device state
        /// @param state The new device state
        void set_device_state(SCR::DeviceState state);

        /// @brief Set the mobility state
        /// @param mobility The new mobility state
        void set_mobility(bool mobility);

        /// @brief Begin a performance measurement
        /// @param name The name of the performance measurement
        void perf_start(std::string name);

        /// @brief End a performance measurement
        /// @param name The name of the performance measurement
        void perf_end(std::string name);

        /// @brief Runs the node with the correct ROS parameters and specifications
        /// @param node 
        static void run_node(std::shared_ptr<Node> node);

        /// @brief Runs the nodes with the correct ROS parameters and specifications
        /// @param nodes
        static void run_nodes(std::vector<std::shared_ptr<Node>> nodes);

    protected:
        /// @brief Called after a node is first discovered by the network. The device state will be set to BOOTING
        virtual void init() = 0;

        /// @brief Called after any update to the system state (state, mode, or mobility)
        /// @param old The previous system state
        /// @param updated The updated system state
        virtual void system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) = 0;

        /// @brief Called when any update is made to the clients configuration
        /// @param config The updated configuration
        virtual void config_updated(json config) = 0;

        /// @brief Returns the default current configuration of the node
        /// @return The default configuration
        virtual json get_default_config() = 0;
        
        /// @brief Logs a message to the logging topic
        /// @param data The message to log
        void log(std::string data);

        /// @brief Logs a message to the console
        /// @param message The message to log
        void log_debug(std::string message);

    private:
        void system_state_callback(const scr_msgs::msg::SystemState msg);
        void device_state_callback(const scr_msgs::msg::DeviceState msg);
        void config_updated_callback(const scr_msgs::msg::ConfigUpdated msg);
        void set_system_total_state(SCR::SystemState state, SCR::SystemMode mode, bool mobility);

    protected:
        /// @brief The current system mode
        SCR::SystemMode system_mode = SCR::SystemMode::COMPETITION;

        /// @brief The current system state
        SCR::SystemState system_state = SCR::SystemState::DISABLED;

        /// @brief The current device state
        SCR::DeviceState device_state = SCR::DeviceState::OFF;

        /// @brief The nodes identifier/name
        std::string identifier;

        /// @brief The current mobility state
        bool mobility;

        /// @brief The current map of performance measurements
        std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> perf_measurements;

        /// @brief The current map of device configurations
        std::map<std::string, SCR::DeviceState> device_states;

        /// @brief The current qos profile
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    private:
        NodeSubscriptions subscriptions;
        NodePublishers publishers;
        NodeClients clients;
        CallbackGroups callback_groups;
    };
}
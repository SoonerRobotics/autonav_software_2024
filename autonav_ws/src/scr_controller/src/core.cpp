#include "scr_msgs/srv/update_system_state.hpp"
#include "scr_msgs/msg/device_state.hpp"
#include "scr_msgs/msg/config_updated.hpp"
#include "scr_msgs/msg/system_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scr/constants.hpp"
#include "scr/states.hpp"
#include "scr/structs.hpp"
#include "scr_msgs/srv/update_device_state.hpp"
#include "scr_msgs/srv/update_config.hpp"
#include <chrono>

#include "scr/json.hpp"
using json = nlohmann::json;

struct publishers
{
    rclcpp::Publisher<scr_msgs::msg::SystemState>::SharedPtr system_state;
    rclcpp::Publisher<scr_msgs::msg::DeviceState>::SharedPtr device_state;
    rclcpp::Publisher<scr_msgs::msg::ConfigUpdated>::SharedPtr config_updated;
};

struct services
{
    rclcpp::Service<scr_msgs::srv::UpdateSystemState>::SharedPtr system_state;
    rclcpp::Service<scr_msgs::srv::UpdateDeviceState>::SharedPtr device_state;
    rclcpp::Service<scr_msgs::srv::UpdateConfig>::SharedPtr config_update;
};

class CoreNode : public rclcpp::Node
{
public:
    CoreNode() : Node("scr_core")
    {
        publishers.system_state = this->create_publisher<scr_msgs::msg::SystemState>(SCR::Constants::Topics::SYSTEM_STATE, 10);
        publishers.device_state = this->create_publisher<scr_msgs::msg::DeviceState>(SCR::Constants::Topics::DEVICE_STATE, 10);
        publishers.config_updated = this->create_publisher<scr_msgs::msg::ConfigUpdated>(SCR::Constants::Topics::CONFIG_UPDATE, 10);

        services.system_state = this->create_service<scr_msgs::srv::UpdateSystemState>(SCR::Constants::Services::SYSTEM_STATE, std::bind(&CoreNode::on_system_state_called, this, std::placeholders::_1, std::placeholders::_2));
        services.device_state = this->create_service<scr_msgs::srv::UpdateDeviceState>(SCR::Constants::Services::DEVICE_STATE, std::bind(&CoreNode::on_device_state_called, this, std::placeholders::_1, std::placeholders::_2));
        services.config_update = this->create_service<scr_msgs::srv::UpdateConfig>(SCR::Constants::Services::CONFIG_UPDATE, std::bind(&CoreNode::on_config_update_called, this, std::placeholders::_1, std::placeholders::_2));

        state = SCR::SystemState::DISABLED;
        mode = SCR::SystemMode::COMPETITION;
    }

private:
    /// @brief Callback for when the system state service is called. This is used to set the system state.
    /// @param request 
    /// @param response 
    void on_system_state_called(const std::shared_ptr<scr_msgs::srv::UpdateSystemState::Request> request, std::shared_ptr<scr_msgs::srv::UpdateSystemState::Response> response)
    {
        if (request->state < 0 || request->state > (uint8_t)SCR::SystemState::SHUTDOWN)
        {
            RCLCPP_ERROR(this->get_logger(), "Received invalid system state %d", request->state);
            response->success = false;
            return;
        }

        if (request->mode < 0 || request->mode > (uint8_t)SCR::SystemMode::PRACTICE)
        {
            RCLCPP_ERROR(this->get_logger(), "Received invalid system mode %d", request->mode);
            response->success = false;
            return;
        }

        // Update the system state
        state = (SCR::SystemState)request->state;
        mode = (SCR::SystemMode)request->mode;
        mobility = request->mobility;

        // Send the response
        response->success = true;

        // Publish the new system state
        scr_msgs::msg::SystemState system_state_message;
        system_state_message.state = request->state;
        system_state_message.mode = request->mode;
        system_state_message.mobility = request->mobility;
        publishers.system_state->publish(system_state_message);
    }

    /// @brief Callback for when the device state service is called. This is used to set a specific devices state.
    /// @param request 
    /// @param response 
    void on_device_state_called(const std::shared_ptr<scr_msgs::srv::UpdateDeviceState::Request> request, std::shared_ptr<scr_msgs::srv::UpdateDeviceState::Response> response)
    {
        if (request->state < 0 || request->state > (uint8_t)SCR::DeviceState::ERRORED)
        {
            RCLCPP_ERROR(this->get_logger(), "Received invalid device state %d for %s", request->state, request->device.c_str());
            response->success = false;
            return;
        }

        // Update the device state
        RCLCPP_INFO(this->get_logger(), "Device %s state changed to %s", request->device.c_str(), SCR::toString((SCR::DeviceState)request->state).c_str());
        if (device_states.find(request->device) == device_states.end())
        {
            // This is the first time we've seen this device, so we need to publish the system state, device state, and all known configs
            scr_msgs::msg::SystemState system_state_message;
            system_state_message.state = (uint8_t)state;
            system_state_message.mode = (uint8_t)mode;
            system_state_message.mobility = mobility;
            publishers.system_state->publish(system_state_message);

            // Publish all known device states
            for (auto &device_state : device_states)
            {
                scr_msgs::msg::DeviceState device_state_message;
                device_state_message.device = device_state.first;
                device_state_message.state = (uint8_t)device_state.second;
                publishers.device_state->publish(device_state_message);
            }

            // Publish all known configs
            for (auto &config : configs)
            {
                scr_msgs::msg::ConfigUpdated config_updated_message;
                config_updated_message.device = config.first;
                config_updated_message.json = config.second.dump();
                publishers.config_updated->publish(config_updated_message);
            }
        }
        device_states[request->device] = (SCR::DeviceState)request->state;

        // Send the response
        response->success = true;

        // Publish the new device state
        scr_msgs::msg::DeviceState device_state_message;
        device_state_message.device = request->device;
        device_state_message.state = request->state;
        publishers.device_state->publish(device_state_message);
    }

    void on_config_update_called(const std::shared_ptr<scr_msgs::srv::UpdateConfig::Request> request, std::shared_ptr<scr_msgs::srv::UpdateConfig::Response> response)
    {
        json config;
        try
        {
            config = json::parse(request->json);
        }
        catch (json::parse_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse config update: %s", e.what());
            response->success = false;
            return;
        }

        // Store the config
        configs[request->device] = config;

        // Send the response
        response->success = true;

        // Publish the new config
        RCLCPP_INFO(this->get_logger(), "Config updated for device %s -> %s", request->device.c_str(), request->json.c_str());
        scr_msgs::msg::ConfigUpdated config_updated_message;
        config_updated_message.device = request->device;
        config_updated_message.json = request->json;
        publishers.config_updated->publish(config_updated_message);
    }

private:
    struct publishers publishers;
    struct services services;

    SCR::SystemState state;
    SCR::SystemMode mode;
    bool mobility;
    std::map<std::string, SCR::DeviceState> device_states;
    std::map<std::string, json> configs;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoreNode>());
    rclcpp::shutdown();
}
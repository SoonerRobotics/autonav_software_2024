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
#include "scr_msgs/srv/set_active_preset.hpp"
#include "scr_msgs/srv/save_active_preset.hpp"
#include "scr_msgs/srv/get_presets.hpp"
#include "scr_msgs/srv/delete_preset.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <signal.h>

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
    rclcpp::Service<scr_msgs::srv::SetActivePreset>::SharedPtr set_active_preset;
    rclcpp::Service<scr_msgs::srv::SaveActivePreset>::SharedPtr save_active_preset;
    rclcpp::Service<scr_msgs::srv::GetPresets>::SharedPtr get_presets;
    rclcpp::Service<scr_msgs::srv::DeletePreset>::SharedPtr delete_preset;
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
        services.set_active_preset = this->create_service<scr_msgs::srv::SetActivePreset>(SCR::Constants::Services::SET_ACTIVE_PRESET, std::bind(&CoreNode::on_set_active_preset_called, this, std::placeholders::_1, std::placeholders::_2));
        services.save_active_preset = this->create_service<scr_msgs::srv::SaveActivePreset>(SCR::Constants::Services::SAVE_ACTIVE_PRESET, std::bind(&CoreNode::on_save_active_preset_called, this, std::placeholders::_1, std::placeholders::_2));
        services.get_presets = this->create_service<scr_msgs::srv::GetPresets>(SCR::Constants::Services::GET_PRESETS, std::bind(&CoreNode::on_get_presets_called, this, std::placeholders::_1, std::placeholders::_2));
        services.delete_preset = this->create_service<scr_msgs::srv::DeletePreset>(SCR::Constants::Services::DELETE_PRESET, std::bind(&CoreNode::on_delete_preset_called, this, std::placeholders::_1, std::placeholders::_2));

        // Load the initial system state from the parameters
        mode = static_cast<SCR::SystemMode>(this->declare_parameter<int>("mode", static_cast<int>(SCR::SystemMode::COMPETITION)));
        state = static_cast<SCR::SystemState>(this->declare_parameter<int>("state", static_cast<int>(SCR::SystemState::DISABLED)));
        mobility = this->declare_parameter<bool>("mobility", false);

        // Set mode timer
        mode_timer = this->create_wall_timer(std::chrono::seconds(5), std::bind(&CoreNode::mode_timer_callback, this));

        // god help me
    }

private:
    std::string get_preset_by_name(std::string name)
    {
        // Look at $HOME/.config/autonav/{name}.preset
        std::string home = std::getenv("HOME");
        std::string path = home + "/.config/autonav/" + name + ".preset";
        if (!std::filesystem::exists(path))
        {
            return "";
        }

        std::ifstream file(path);
        std::string preset((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        return preset;
    }

    std::string get_preset_for_mode()
    {
        std::string modeStr = SCR::systemModeToString(mode);
        return get_preset_by_name(modeStr);
    }

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

        if (state == SCR::SystemState::SHUTDOWN)
        {
            // Start a 5 second timer to kill the node
            mode_timer = this->create_wall_timer(std::chrono::seconds(5), std::bind(&CoreNode::kill_self, this));
        }

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

            // Reset the mode timer
            if (mode_timer_running)
            {
                mode_timer->reset();
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

    void ensure_directories()
    {
        std::string home = std::getenv("HOME");
        std::string path = home + "/.config/autonav/";
        if (!std::filesystem::exists(path))
        {
            std::filesystem::create_directories(path);
        }
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

        // Publish the new config
        publish_config(request->device, config);

        // Send the response
        response->success = true;
    }

    void on_set_active_preset_called(const std::shared_ptr<scr_msgs::srv::SetActivePreset::Request> request, std::shared_ptr<scr_msgs::srv::SetActivePreset::Response> response)
    {
        ensure_directories();
        std::string preset = get_preset_by_name(request->preset);
        if (preset.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load preset %s", request->preset.c_str());
            response->ok = false;
            return;
        }

        active_preset = json::parse(preset);
        active_preset_name = request->preset;
        for (auto const &[device, cfg] : active_preset)
        {
            if (configs.find(device) == configs.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Warning when loading preset %s: missing config for device %s", request->preset.c_str(), device.c_str());
                continue;
            }

            // Update the config
            configs[device] = cfg;

            // Publish the new config
            publish_config(device, cfg);
        }

        response->ok = true;
    }

    void on_save_active_preset_called(const std::shared_ptr<scr_msgs::srv::SaveActivePreset::Request> request, std::shared_ptr<scr_msgs::srv::SaveActivePreset::Response> response)
    {
        ensure_directories();
        active_preset = configs;
        active_preset_name = request->write_mode ? SCR::systemModeToString(mode) : request->preset_name;

        std::string name = request->write_mode ? SCR::systemModeToString(mode) : request->preset_name;
        std::string home = std::getenv("HOME");
        std::string path = home + "/.config/autonav/" + name + ".preset";

        write_preset(name, active_preset);
        response->ok = true;
    }

    void on_get_presets_called(const std::shared_ptr<scr_msgs::srv::GetPresets::Request> request, std::shared_ptr<scr_msgs::srv::GetPresets::Response> response)
    {
        ensure_directories();
        std::string home = std::getenv("HOME");
        std::string path = home + "/.config/autonav/";

        // Get all files that end with .preset
        presets.clear();
        for (const auto &entry : std::filesystem::directory_iterator(path))
        {
            std::string filename = entry.path().filename().string();
            if (filename.find(".preset") != std::string::npos)
            {
                presets.push_back(filename.substr(0, filename.find(".preset")));
            }
        }

        response->presets = presets;
        response->active_preset = active_preset_name;
    }

    void mode_timer_callback()
    {
        ensure_directories();

        // Stop the mode timer
        mode_timer_running = false;
        mode_timer->cancel();
        mode_timer = nullptr;

        // Check if there is a preset for the current mode
        std::string preset = get_preset_for_mode();
        if (!preset.empty())
        {
            auto upcoming_preset = json::parse(preset);

            // Checking for missing devices and keys
            for (auto const &[device, cfg] : configs)
            {
                // Check for missing devices
                if (upcoming_preset.find(device) == upcoming_preset.end())
                {
                    // The upcoming preset is missing a config, so we need to add it
                    upcoming_preset[device] = cfg;
                    RCLCPP_WARN(this->get_logger(), "Warning: upcoming preset is missing config for device %s", device.c_str());
                    continue;
                }

                // Check for missing keys
                for (auto const &[key, value] : cfg.items())
                {
                    if (upcoming_preset[device].find(key) == upcoming_preset[device].end())
                    {
                        // The upcoming preset is missing a key, so we need to add it
                        upcoming_preset[device][key] = value;
                        RCLCPP_WARN(this->get_logger(), "Warning: upcoming preset is missing key %s for device %s", key.c_str(), device.c_str());
                    }
                }
            }

            active_preset = upcoming_preset;
            active_preset_name = SCR::systemModeToString(mode);
            for (auto const &[device, cfg] : active_preset)
            {
                if (configs.find(device) == configs.end())
                {
                    continue;
                }

                // Update the config
                configs[device] = cfg;

                // Publish the new config
                publish_config(device, cfg);
            }

            write_preset(SCR::systemModeToString(mode), active_preset);
            return;
        }

        // If there is no preset for the current mode, then we need to create one
        active_preset = configs;
        active_preset_name = SCR::systemModeToString(mode);
        write_preset(active_preset_name, active_preset);
        RCLCPP_INFO(this->get_logger(), "Created preset for mode %d", (int)mode);
    }

    void on_delete_preset_called(const std::shared_ptr<scr_msgs::srv::DeletePreset::Request> request, std::shared_ptr<scr_msgs::srv::DeletePreset::Response> response)
    {
        ensure_directories();
        
        // Check that the preset exists
        std::string home = std::getenv("HOME");
        std::string path = home + "/.config/autonav/" + request->preset + ".preset";
        if (!std::filesystem::exists(path))
        {
            response->ok = false;
            return;
        }

        // Ensure its not the active mode preset
        if (request->preset == SCR::systemModeToString(mode))
        {
            response->ok = false;
            return;
        }

        // Delete the preset
        std::filesystem::remove(path);

        // Remove it from the list of presets
        auto it = std::find(presets.begin(), presets.end(), request->preset);
        if (it != presets.end())
        {
            presets.erase(it);
        }

        // If it was the active preset, change the active preset to the current system mode
        if (active_preset_name == request->preset)
        {
            active_preset = json::parse(get_preset_for_mode());
            active_preset_name = SCR::systemModeToString(mode);
            for (auto const &[device, cfg] : active_preset)
            {
                if (configs.find(device) == configs.end())
                {
                    continue;
                }

                // Update the config
                configs[device] = cfg;

                // Publish the new config
                publish_config(device, cfg);
            }
        }

        response->ok = true;
    }

    void write_preset(std::string name, json preset)
    {
        std::string home = std::getenv("HOME");
        std::string path = home + "/.config/autonav/" + name + ".preset";

        // Write the preset to disk, if it exists then overwrite it
        std::string jsonStr = nlohmann::json(preset).dump();
        std::ofstream file(path);
        file << jsonStr;
        file.close();
    }

    void publish_config(std::string device, nlohmann::json cfg)
    {
        scr_msgs::msg::ConfigUpdated config_updated_message;
        config_updated_message.device = device;
        config_updated_message.json = cfg.dump();
        publishers.config_updated->publish(config_updated_message);
    }

    void kill_self()
    {
        kill(getpid(), SIGKILL);
    }

private:
    struct publishers publishers;
    struct services services;

    SCR::SystemState state;
    SCR::SystemMode mode;
    bool mobility;
    std::map<std::string, SCR::DeviceState> device_states;
    std::map<std::string, json> configs;
    std::map<std::string, json> active_preset;
    std::vector<std::string> presets;
    rclcpp::TimerBase::SharedPtr mode_timer;
    bool mode_timer_running = true;
    std::string active_preset_name;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoreNode>());
    rclcpp::shutdown();
}
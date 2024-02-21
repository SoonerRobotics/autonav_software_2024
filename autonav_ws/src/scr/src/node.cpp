#include "scr/node.hpp"
#include <unistd.h>

namespace SCR
{
    Node::Node(std::string node_name) : rclcpp::Node(node_name)
    {
        identifier = node_name;

        // Create the callback groups
        callback_groups.device_state = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_groups.system_state = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_groups.config_updated = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create subscriptions
        subscriptions.system_state = this->create_subscription<scr_msgs::msg::SystemState>(Constants::Topics::SYSTEM_STATE, 10, std::bind(&Node::system_state_callback, this, std::placeholders::_1));
        subscriptions.device_state = this->create_subscription<scr_msgs::msg::DeviceState>(Constants::Topics::DEVICE_STATE, 10, std::bind(&Node::device_state_callback, this, std::placeholders::_1));
        subscriptions.config_updated = this->create_subscription<scr_msgs::msg::ConfigUpdated>(Constants::Topics::CONFIG_UPDATE, 10, std::bind(&Node::config_updated_callback, this, std::placeholders::_1));

        // Create publishers
        publishers.performance_track = this->create_publisher<std_msgs::msg::Float64>(Constants::Topics::PERFORMANCE_TRACK, 10);

        // Create clients
        clients.system_state = this->create_client<scr_msgs::srv::UpdateSystemState>(Constants::Services::SYSTEM_STATE, rmw_qos_profile_services_default, callback_groups.system_state);
        clients.device_state = this->create_client<scr_msgs::srv::UpdateDeviceState>(Constants::Services::DEVICE_STATE, rmw_qos_profile_services_default, callback_groups.device_state);
        clients.config_update = this->create_client<scr_msgs::srv::UpdateConfig>(Constants::Services::CONFIG_UPDATE, rmw_qos_profile_services_default, callback_groups.config_updated);

        // Create a thread to wait a sec for the node to boot without blocking the main thread
        std::thread booting_thread([this]()
                                   {
            sleep(1);
            set_device_state(SCR::DeviceState::BOOTING); });
        booting_thread.detach();
    }

    Node::~Node()
    {
    }

    void Node::system_state_callback(const scr_msgs::msg::SystemState msg)
    {
        scr_msgs::msg::SystemState oldState;
        oldState.state = system_state;
        oldState.mobility = mobility;
        oldState.mode = system_mode;

        SCR::SystemState newStateEnum = static_cast<SCR::SystemState>(msg.state);
        SCR::SystemState oldStateEnum = static_cast<SCR::SystemState>(oldState.state);

        SCR::SystemMode newMode = static_cast<SCR::SystemMode>(msg.mode);
        SCR::SystemMode oldMode = static_cast<SCR::SystemMode>(oldState.mode);

        RCLCPP_WARN(this->get_logger(), "SYSTEM STATE CHANGE: %s -> %s", SCR::toString(oldStateEnum).c_str(), SCR::toString(newStateEnum).c_str());
        RCLCPP_WARN(this->get_logger(), "SYSTEM MODE CHANGE: %s -> %s", SCR::toString(oldMode).c_str(), SCR::toString(newMode).c_str());
        RCLCPP_WARN(this->get_logger(), "MOBILITY CHANGE: %d -> %d", oldState.mobility, msg.mobility);

        system_state_transition(oldState, msg);

        system_mode = static_cast<SCR::SystemMode>(msg.mode);
        mobility = msg.mobility;
        system_state = static_cast<SCR::SystemState>(msg.state);

        if (system_state == SCR::SystemState::SHUTDOWN)
        {
            // Just exit this process and die
            exit(0);
        }
    }

    void Node::device_state_callback(const scr_msgs::msg::DeviceState msg)
    {
        device_states[msg.device] = static_cast<SCR::DeviceState>(msg.state);
        if (msg.device != identifier)
        {
            return;
        }

        device_state = static_cast<SCR::DeviceState>(msg.state);
        RCLCPP_INFO(this->get_logger(), "Received Topic: Device state changed to %s", SCR::toString(device_state).c_str());

        if (device_state == SCR::DeviceState::BOOTING)
        {
            // Get the default config and push it to the server
            json config = get_default_config();
            auto request = std::make_shared<scr_msgs::srv::UpdateConfig::Request>();
            request->device = identifier;
            request->json = config.dump();

            while (!clients.config_update->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for config_update service. Exiting.");
                    return;
                }
            }

            auto result_future = clients.config_update->async_send_request(request);
            std::future_status status = result_future.wait_for(std::chrono::seconds(1));
            if (status != std::future_status::ready)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call config_update service");
                return;
            }

            auto result = result_future.get();
            if (!result)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to update config");
            }

            // We are booted and have a connection to the server, lets go into standby and call our own init function
            config_updated(config);
            set_device_state(SCR::DeviceState::STANDBY);
            init();
            return;
        }
    }

    void Node::config_updated_callback(const scr_msgs::msg::ConfigUpdated msg)
    {
        if (msg.device != identifier)
        {
            return;
        }

        try
        {
            json config = json::parse(msg.json);
            config_updated(config);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse config: %s", e.what());
        }
    }

    void Node::set_device_state(SCR::DeviceState state)
    {
        if (state == device_state)
        {
            return;
        }

        auto request = std::make_shared<scr_msgs::srv::UpdateDeviceState::Request>();
        request->device = identifier;
        request->state = state;

        while (!clients.device_state->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for device_state service. Exiting.");
                return;
            }
        }

        auto result_future = clients.device_state->async_send_request(request);
        std::future_status status = result_future.wait_for(std::chrono::seconds(1));
        if (status != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call device_state service");
            return;
        }

        auto result = result_future.get();
        if (!result)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set device state to: %s", SCR::toString(state).c_str());
        }
    }

    void Node::set_system_total_state(SCR::SystemState state, SCR::SystemMode mode, bool mobility)
    {
        auto request = std::make_shared<scr_msgs::srv::UpdateSystemState::Request>();
        request->state = state;
        request->mode = mode;
        request->mobility = mobility;

        while (!clients.system_state->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for system_state service. Exiting.");
                return;
            }
        }

        auto result_future = clients.system_state->async_send_request(request);
        std::future_status status = result_future.wait_for(std::chrono::seconds(1));
        if (status != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call system_state service");
            return;
        }

        auto result = result_future.get();
        if (!result)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to update system state");
        }
    }

    void Node::set_system_mode(SCR::SystemMode nMode)
    {
        set_system_total_state(system_state, nMode, mobility);
    }

    void Node::set_system_state(SCR::SystemState nState)
    {
        set_system_total_state(nState, system_mode, mobility);
    }

    void Node::set_mobility(bool nMobility)
    {
        set_system_total_state(system_state, system_mode, nMobility);
    }

    void Node::perf_start(std::string name)
    {
        perf_measurements[name] = std::chrono::high_resolution_clock::now();
    }

    void Node::perf_end(std::string name)
    {
        if (perf_measurements.find(name) == perf_measurements.end())
        {
            RCLCPP_ERROR(this->get_logger(), "There was no performance measurement with the name '%s'", name.c_str());
            return;
        }

        auto start = perf_measurements[name];
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std_msgs::msg::Float64 msg;
        msg.data = duration_ms;
        publishers.performance_track->publish(msg);

        perf_measurements.erase(name);
    }

    void Node::run_node(std::shared_ptr<Node> node)
    {
        // Create a multi-threaded executor
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);

        // Run the executor
        executor.spin();

        // Shutdown
        executor.remove_node(node);

        rclcpp::shutdown();
    }

    void Node::run_nodes(std::vector<std::shared_ptr<Node>> nodes)
    {
        rclcpp::executors::MultiThreadedExecutor executor;
        for (auto node : nodes)
        {
            executor.add_node(node);
        }

        executor.spin();

        for (auto node : nodes)
        {
            executor.remove_node(node);
        }

        rclcpp::shutdown();
    }

}
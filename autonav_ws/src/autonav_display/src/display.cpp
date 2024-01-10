#include "scr/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scr/websocketpp/config/asio_no_tls.hpp"
#include "scr/websocketpp/server.hpp"
#include <iostream>

typedef websocketpp::server<websocketpp::config::asio> server;

struct DisplayNodeConfig
{
    uint16_t port;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(DisplayNodeConfig, port)
};

class DisplayNode : public SCR::Node
{
public:
    DisplayNode() : SCR::Node("autonav_display") {}
    ~DisplayNode() {}

    void init() override
    {
        // Setup subscriptions
        device_state_subscription = this->create_subscription<scr_msgs::msg::DeviceState>(SCR::Constants::Topics::DEVICE_STATE, 10, std::bind(&DisplayNode::device_state_callback, this, std::placeholders::_1));
        system_state_subscription = this->create_subscription<scr_msgs::msg::SystemState>(SCR::Constants::Topics::SYSTEM_STATE, 10, std::bind(&DisplayNode::system_state_callback, this, std::placeholders::_1));

        // Setup server logging
        m_server.set_access_channels(websocketpp::log::alevel::all);
        m_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        // Initialize Asio
        m_server.init_asio();

        // Register our message handler
        m_server.set_message_handler(std::bind(&DisplayNode::on_message_received, this, std::placeholders::_1, std::placeholders::_2));
        m_server.set_open_handler(std::bind(&DisplayNode::on_open, this, std::placeholders::_1));
        m_server.set_close_handler(std::bind(&DisplayNode::on_close, this, std::placeholders::_1));

        // Listen on port
        RCLCPP_INFO(this->get_logger(), "Listening on port %d", config.port);
        m_server.listen(config.port);

        // Start the server accept loop
        m_server.start_accept();

        // Start the ASIO io_service run loop in a separate thread
        m_server_thread = std::thread(&server::run, &m_server);

        set_device_state(SCR::DeviceState::OPERATING);
    }

    void on_message_received(websocketpp::connection_hdl hdl, server::message_ptr msg)
    {
        json data;
        try
        {
            data = json::parse(msg->get_payload());
        }
        catch (json::parse_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON from incoming message: %s", e.what());
            return;
        }

        // TODO: Handle json
    }

    void send_data(websocketpp::connection_hdl hdl, std::string topic, json data)
    {
        json packet;
        packet["op"] = "data";
        packet["topic"] = topic;
        for (auto it = data.begin(); it != data.end(); ++it)
        {
            packet[it.key()] = it.value();
        }

        m_server.send(hdl, packet.dump(), websocketpp::frame::opcode::text);
    }

    void broadcast_data(std::string topic, json data)
    {
        for (auto it : connections)
        {
            send_data(it, topic, data);
        }
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        // Send the current system state
        json data = {
            {"state", (uint8_t)SCR::SystemState::DISABLED},
            {"mode", (uint8_t)SCR::SystemMode::COMPETITION},
            {"mobility", 0}};
        send_data(hdl, "/scr/state/system", data);

        // Send the current known device states
        for (auto [device, state] : device_states)
        {
            json data = {
                {"device", device},
                {"state", (uint8_t)state}};
            send_data(hdl, "/scr/state/device", data);
        }

        // Add the connection to the list
        connections.insert(hdl);
    }

    void on_close(websocketpp::connection_hdl hdl)
    {
        // Remove the connection from the list
        connections.erase(hdl);
    }

    void device_state_callback(const scr_msgs::msg::DeviceState::SharedPtr msg)
    {
        json data = {
            {"device", msg->device},
            {"state", msg->state}};
        broadcast_data("/scr/state/device", data);
    }

    void system_state_callback(const scr_msgs::msg::SystemState::SharedPtr msg)
    {
        json data = {
            {"state", msg->state},
            {"mode", msg->mode},
            {"mobility", msg->mobility}};
        broadcast_data("/scr/state/system", data);
    }

    // Generic callbacks
    void system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
    {
        if (updated.state == SCR::SystemState::SHUTDOWN)
        {
            m_server.stop_listening();
            m_server.stop();
            m_server_thread.join();
        }
    }

    void config_updated(json newConfig) override
    {
        // Print the received port
        RCLCPP_INFO(this->get_logger(), "Received port %d", newConfig["port"].get<uint16_t>());
        config = newConfig.template get<DisplayNodeConfig>();
    }

    json get_default_config() override
    {
        DisplayNodeConfig defaultConfig;
        defaultConfig.port = 8023;
        return defaultConfig;
    }

private:
    DisplayNodeConfig config;

    rclcpp::Subscription<scr_msgs::msg::DeviceState>::SharedPtr device_state_subscription;
    rclcpp::Subscription<scr_msgs::msg::SystemState>::SharedPtr system_state_subscription;

    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections;

public:
    std::thread m_server_thread;
    server m_server;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayNode>();
    SCR::Node::run_node(node);
    rclcpp::shutdown();

    node->m_server.stop_listening();
    node->m_server.stop();
    node->m_server_thread.join();

    return 0;
}
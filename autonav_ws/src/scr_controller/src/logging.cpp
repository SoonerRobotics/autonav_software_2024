#include "scr/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scr_msgs/msg/log.hpp"
#include <fstream>
#include <chrono>

class LoggingNode : public SCR::Node
{
public:
    LoggingNode() : SCR::Node("scr_logging") {}
    ~LoggingNode() {}

    void init() override
    {
        log_subscriber = create_subscription<scr_msgs::msg::Log>("/scr/logging", 20, std::bind(&LoggingNode::on_log_received, this, std::placeholders::_1));
        set_device_state(SCR::DeviceState::OPERATING);

        boot_time = std::to_string(get_current_time_ms().count());
    }

    std::chrono::milliseconds get_current_time_ms()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    }

    void on_log_received(const scr_msgs::msg::Log::SharedPtr msg)
    {
        // Get the home directory
        std::string home_dir = std::getenv("HOME");

        // Create the directory {home}/autonav_logs/{boot_time}
        std::string log_dir = home_dir + "/autonav_logs/" + boot_time;
        if (!std::filesystem::exists(log_dir))
        {
            std::filesystem::create_directories(log_dir);
        }

        // Create the file {home}/autonav_logs/{boot_time}/{node}.csv if it does not exist
        std::string log_file = log_dir + "/" + msg->node + ".csv";
        if (!std::filesystem::exists(log_file))
        {
            std::ofstream file(log_file);
            file << "timestamp,data" << std::endl;
            file.close();
        }

        // Append the log message to the file
        std::chrono::milliseconds current_time = get_current_time_ms();
        std::string log_entry = std::to_string(current_time.count()) + "," + msg->data;
        std::ofstream file(log_file, std::ios_base::app);
        file << log_entry << std::endl;
        file.close();
    }

    void config_updated(json newConfig) override
    {
    }

    json get_default_config() override
    {
        return {};
    }

    void system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
    {
    }

private:
    rclcpp::Subscription<scr_msgs::msg::Log>::SharedPtr log_subscriber;
    std::string boot_time;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<LoggingNode>());
    rclcpp::shutdown();
    return 0;
}
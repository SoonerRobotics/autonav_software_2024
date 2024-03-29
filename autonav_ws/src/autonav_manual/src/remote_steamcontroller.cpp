#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/steam_input.hpp"
#include "std_msgs/msg/float32.hpp"
#include "scr/utils.hpp"
#include "scr/node.hpp"

long lastMessageTime = 0;
float lastForwardSpeed = 0;
float lastTurnSpeed = 0;

struct SteamJoyNodeConfig
{
    float throttle_deadzone;
    float steering_deadzone;
    float forward_speed;
    float turn_speed;
    float max_turn_speed;
    float max_forward_speed;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SteamJoyNodeConfig, throttle_deadzone, steering_deadzone, forward_speed, turn_speed, max_turn_speed, max_forward_speed)
};

class SteamJoyNode : public SCR::Node
{
public:
    SteamJoyNode() : SCR::Node("autonav_manual_steam")
    {
        steam_subscription = create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&SteamJoyNode::onSteamDataReceived, this, std::placeholders::_1));
        motor_publisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20);
    }

    void init() override
    {
        set_device_state(SCR::DeviceState::READY);
    }

    void config_updated(json newConfig) override
    {
        config = newConfig.template get<SteamJoyNodeConfig>();
    }

    json get_default_config() override
    {
        SteamJoyNodeConfig defaultConfig;
        defaultConfig.throttle_deadzone = 0.03f;
        defaultConfig.steering_deadzone = 0.03f;
        defaultConfig.forward_speed = 1.8f;
        defaultConfig.turn_speed = 1.0f;
        defaultConfig.max_turn_speed = 3.14159265f;
        defaultConfig.max_forward_speed = 2.2f;
        return defaultConfig;
    }

    void system_state_transition(scr_msgs::msg::SystemState old_state, scr_msgs::msg::SystemState new_state) override
    {
        if (new_state.state == SCR::SystemState::MANUAL && device_state == SCR::DeviceState::READY)
        {
            set_device_state(SCR::DeviceState::OPERATING);
        }

        if (new_state.state != SCR::SystemState::MANUAL && device_state == SCR::DeviceState::OPERATING)
        {
            set_device_state(SCR::DeviceState::READY);
        }
    }

    void onSteamDataReceived(const autonav_msgs::msg::SteamInput &msg)
    {
        lastMessageTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (system_state != SCR::SystemState::MANUAL || device_state != SCR::DeviceState::OPERATING)
        {
            return;
        }

        float throttle = 0;
        float steering = 0;
        const float throttleDeadzone = config.throttle_deadzone;
        const float steeringDeadzone = config.steering_deadzone;

        if (abs(msg.ltrig) > throttleDeadzone || abs(msg.rtrig) > throttleDeadzone)
        {
            throttle = msg.rtrig;
            throttle = throttle - msg.ltrig;
        }

        if (abs(msg.lpad_x) > steeringDeadzone)
        {
            steering = msg.lpad_x;
        }

        autonav_msgs::msg::MotorInput input;
        input.forward_velocity = SCR::Utilities::clamp(throttle * config.forward_speed, -config.max_forward_speed, config.max_forward_speed);
        input.angular_velocity = -1 * SCR::Utilities::clamp(steering * config.turn_speed, -config.max_turn_speed, config.max_turn_speed);
        motor_publisher->publish(input);
    }

    rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motor_publisher;
    rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr steam_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription;
    rclcpp::TimerBase::SharedPtr heartbeat;

private:
    SteamJoyNodeConfig config;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<SteamJoyNode>());
    rclcpp::shutdown();
    return 0;
}

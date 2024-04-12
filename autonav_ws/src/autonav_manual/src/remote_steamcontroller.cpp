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
    bool invert_throttle;
    bool invert_steering;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SteamJoyNodeConfig, throttle_deadzone, steering_deadzone, forward_speed, turn_speed, max_turn_speed, max_forward_speed, invert_throttle, invert_steering)
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
        defaultConfig.throttle_deadzone = 0.15f;
        defaultConfig.steering_deadzone = 0.15f;
        defaultConfig.forward_speed = 1.8f;
        defaultConfig.turn_speed = 1.0f;
        defaultConfig.max_turn_speed = 3.14159265f;
        defaultConfig.max_forward_speed = 2.2f;
        defaultConfig.invert_steering = true;
        defaultConfig.invert_throttle = true;
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
    
    float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
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
            target_throttle = msg.rtrig - msg.ltrig;
            is_working = true;
        }

        if (abs(msg.lpad_x) > steeringDeadzone)
        {
            target_steering = msg.lpad_x;
            is_working = true;
        }

        if (abs(msg.ltrig) < throttleDeadzone && abs(msg.rtrig) < throttleDeadzone)
        {
            target_throttle = 0;
            is_working = false;
        }

        if (abs(msg.lpad_x) < steeringDeadzone)
        {
            target_steering = 0;
            is_working = false;
        }

        // Generate a forward/angular velocity command that ramps up/down smoothly
        const float throttleRate = 0.03;
        const float steeringRate = 0.01;
        current_throttle = lerp(current_throttle, target_throttle * config.forward_speed, throttleRate * (is_working ? 1 : 1.8));
        current_steering = lerp(current_steering, target_steering * config.turn_speed, steeringRate * (is_working ? 1 : 1.8));

        autonav_msgs::msg::MotorInput input;
        // input.forward_velocity = SCR::Utilities::clamp(throttle * config.forward_speed, -config.max_forward_speed, config.max_forward_speed);
        // input.angular_velocity = -1 * SCR::Utilities::clamp(steering * config.turn_speed, -config.max_turn_speed, config.max_turn_speed);
        input.forward_velocity = SCR::Utilities::clamp(current_throttle, -config.max_forward_speed, config.max_forward_speed);
        input.angular_velocity = SCR::Utilities::clamp(current_steering * config.turn_speed, -config.max_turn_speed, config.max_turn_speed);
        input.forward_velocity *= config.invert_throttle ? -1 : 1;
        input.angular_velocity *= config.invert_steering ? -1 : 1;
        motor_publisher->publish(input);
    }

    rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motor_publisher;
    rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr steam_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription;
    rclcpp::TimerBase::SharedPtr heartbeat;

private:
    SteamJoyNodeConfig config;
    float target_throttle = 0;
    float target_steering = 0;
    float current_throttle = 0;
    float current_steering = 0;
    bool is_working = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<SteamJoyNode>());
    rclcpp::shutdown();
    return 0;
}

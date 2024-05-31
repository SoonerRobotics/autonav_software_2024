#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "scr/node.hpp"
#include "scr/states.hpp"

#define MAX_SPEED 1.4

class JoyXboxNode : public SCR::Node
{
public:
    JoyXboxNode() : SCR::Node("autonav_manual_xbox") {}

    void init() override
    {
        steamSubscription = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyXboxNode::onJoyReceived, this, std::placeholders::_1));
        motorPublisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 10);

        set_device_state(SCR::DeviceState::READY);
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

private:
    void onJoyReceived(const sensor_msgs::msg::Joy &msg) const
    {
        float throttle = 0;
        float steering = 0;

        autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
        if (abs(msg.axes[5]) > 0.05 || abs(msg.axes[2]) > 0.05)
        {
            throttle = (1 - msg.axes[5]) * MAX_SPEED * 0.8;
            throttle = throttle - (1 - msg.axes[2]) * MAX_SPEED * 0.8;
        }

        if (abs(msg.axes[0]) > 0.15)
        {
            steering = msg.axes[0] * MAX_SPEED;
        }

        package.forward_velocity = throttle;
        package.angular_velocity = steering;
        motorPublisher->publish(package);
    }
    
    void config_updated(json config) override
    {
    }

    json get_default_config() override
    {
        return json();
    }

    rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motorPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr steamSubscription;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<JoyXboxNode>());
    rclcpp::shutdown();
    return 0;
}
#include "autonav_core/core.h"
#include <unistd.h>

namespace SCR
{
    AutonavNode::AutonavNode(std::string node_name) : rclcpp::Node(node_name)
    {
        robot_state_subscription = this->create_subscription<autonav_msgs::msg::RobotState>(
        "autonav/get_robot_state", 10, std::bind(&AutonavNode::RobotStateUpdate, this, std::placeholders::_1));
    
        device_state_subscription = this->create_subscription<autonav_msgs::msg::DeviceState>(
        "autonav/get_device_state", 10, std::bind(&AutonavNode::DeviceStateUpdate, this, std::placeholders::_1));
        
        robot_state_publisher = this->create_publisher<autonav_msgs::msg::RobotState>("autonav/set_robot_state", 20);
    }

    AutonavNode::~AutonavNode()
    {
    }

    void AutonavNode::RobotStateUpdate(autonav_msgs::msg::RobotState::SharedPtr robot_state_msg) {
        this->robot_state = SCR::RobotState(robot_state_msg->value);
    }

    void AutonavNode::DeviceStateUpdate(autonav_msgs::msg::DeviceState::SharedPtr msg) {
        if (msg->node_name == this->get_name()) {
            this->device_state = SCR::DeviceState(msg->value);
        }
    }

    void AutonavNode::UpdateRobotState(autonav_msgs::msg::RobotState::SharedPtr robot_state_msg) {
        robot_state_publisher->publish(*robot_state_msg);
    }
}
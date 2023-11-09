#include "autonav_core/robot_state_manager.hpp"

namespace SCR {

    RobotStateManagerNode::RobotStateManagerNode() : Node("robot_state_manager")
    {
        robot_state_subscription = this->create_subscription<autonav_msgs::msg::RobotState>(
        "autonav/get_robot_state", 10, std::bind(&RobotStateManagerNode::robot_state_update, this, std::placeholders::_1));
    
        device_state_subscription = this->create_subscription<autonav_msgs::msg::DeviceState>(
        "autonav/get_device_state", 10, std::bind(&RobotStateManagerNode::device_state_update, this, std::placeholders::_1));
        
        robot_state_publisher = this->create_publisher<autonav_msgs::msg::RobotState>("autonav/set_robot_state", 20);

        device_state_publisher = this->create_publisher<autonav_msgs::msg::DeviceState>("autonav/set_device_state", 20);
    }

    RobotStateManagerNode::~RobotStateManagerNode()
    {
    }

    Device RobotStateManagerNode::get_device_by_name(std::string name) {
        for (Device device : this->devices) {
            if (device.name == name) {
                return device;
            }
        }
    }

    void RobotStateManagerNode::device_state_update(autonav_msgs::msg::DeviceState::SharedPtr device_state_msg) {
        std::vector<Device> list = this->devices;
        // TODO: Routine for assigning adding a new device to the device list

        // TODO: Routine for setting a device state
    }   
}
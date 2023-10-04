#pragma once

#include "rclcpp/rclcpp.hpp"

namespace SCR
{
    class AutonavNode : public rclcpp::Node
    {
    public:
        AutonavNode(std::string node_name);
        ~AutonavNode();
    };
}
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "autonav_core/performance_timer.hpp"


namespace SCR
{
    class AutonavNode : public rclcpp::Node
    {
        public:
            AutonavNode(std::string node_name);
            ~AutonavNode();

        private:
            SCR::PerformanceTimer performance_timer = SCR::PerformanceTimer("default_timer");
    };
}
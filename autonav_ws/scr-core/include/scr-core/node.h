#pragma once


#include "rclcpp/rclcpp.hpp"
#include "string.h"

namespace SCR {

    enum SystemState {
        DISABLED,
        AUTO,
        MANUAL,
        TEST
    };

    enum NodeState {
        DEAD,
        CONFIGURING,
        ALIVE
    };


    class Node : public rclcpp::Node {
        public:
            Node(std::string node_name);
            ~Node();

            // getters
            SystemState GetSystemState();
            NodeState GetNodeState();


            // setters
            void SetSystemState(SystemState state);
            void SetNodeState(NodeState state);
            void OnSystemState(scr_msgs::msg::SystemState msg);
    };

    private:
        NodeState state_;
        SystemState systemState_;

        rclcpp::Subscription<scr_msgs::msg::SystemState>::SharedPtr systemStateSubscriber;
}
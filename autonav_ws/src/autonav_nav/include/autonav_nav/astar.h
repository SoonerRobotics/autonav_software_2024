#pragma once

// C++ includes
#include <string>
#include <iostream>
#include <filesystem>

// SCR::Node
#include "scr/node.hpp"

// ROS messages
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
// #include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"

// JSON stuff (for config)
#include "scr/json.hpp"

// to keep character counts down, 'cause in this economy every letter counts
using json = nlohmann::json;

class AStarNode : public SCR::Node {
public:
    // we don't use constructors here
    AStarNode() : SCR::Node("astar_fast")  {};
    ~AStarNode() {};

    // main init method, 
    void init() override;

    // let us know when we're in AUTONOMOUS so we can start path planning
    void system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override;

    // get the default config (copied and pasted from last year)
    json get_default_config() override;

    // whenever a new config from the UI is given to us
    void config_updated(json newConfig) override;

private:
    // members fields
    //TODO

    // subscribers
    //TODO

    // subscriber callbacks
    //TODO

    // publishers
    //TODO

    // publisher callbacks
    //TODO

    // main methods
    //TODO
};
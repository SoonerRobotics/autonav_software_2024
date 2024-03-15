#pragma once

// C++ includes
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

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


// === structs ===
// struct to represent a node on the graph for the A* algorithm (essentially a square in the grid/map, but like math-technically it's a 'node' or something)
struct GraphNode {
    // x and y coordinates
    int x;
    int y;

    // costs
    double g_cost; // movement cost (distance from start)
    double h_cost; // heueristic (distance from goal)
    double f_cost; // total cost

    // pointer to parent
    GraphNode* parent;

    // define the less_than operator so we can use std::sort on the list
    // https://stackoverflow.com/questions/1380463/sorting-a-vector-of-custom-objects
    bool operator < (const GraphNode& other) const {
        return (f_cost < other.f_cost);
    }

    // two nodes are equal if their coordinates are equal (obviously)
    bool operator == (const GraphNode& other) const {
        return (x == other.x && y == other.y);
    }

    // to_string for debugging
    std::string to_string() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }
};

// pretty basic point struct
struct GPSPoint {
    double lat;
    double lon;
};



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
    // Y and X dimensions for the occpancy grid
    const static int MAX_Y = 80;
    const static int MAX_X = 80;

    // member fields
    int* map[MAX_Y * MAX_X] = {}; // 1D map (row-major) of all grid data
    //https://www.geeksforgeeks.org/priority-queue-in-cpp-stl/ and  https://cplusplus.com/reference/queue/priority_queue/
    std::priority_queue<GraphNode> frontier; // priority queue (aka heap, heapqueue, etc) of all the points we need to explore next for A* (priority queue is used because it is fast and good)
    GraphNode* closed[MAX_X * MAX_Y] = {}; // list of all points we've explored in the current iteration of A*
    GPSPoint position; // position of robot (lat, lon)
    std::vector<GPSPoint> waypoints;

    bool getNewGpsCoords; // whether we need to update the waypoints array or not
    const int MAX_DEPTH = 50; // smellification max depth
    std::vector<GraphNode> smellyFrontier; // smellification frontier

    // stuff for file-reading code
    const std::string WAYPOINTS_FILENAME = "./data/waypoints.csv"; // filename for the waypoints (should be CSV file with label,lat,lon,)
    std::ifstream waypointsFile; // actual C++ file object
    std::unordered_map<std::string, std::vector<GPSPoint>> waypointsDict; // dictionairy of lists containing the GPS waypoints we're AStar-ing to


    // subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr expandedSubscriber; // subscirbes to the output of expandification
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr poseSubscriber; // subscribes to sensor fusion position output
    
    // subscriber callbacks
    void onOccupancyGridReceived(nav_msgs::msg::OccupancyGrid msg);
    void onPositionReceived(geometry_msgs::msg::Pose msg);

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;

    // publisher callbacks
    void publishPath();

    // main methods
    GraphNode getGoalPoint(); // Smellification algorithm
    std::vector<GraphNode> doAStar(); // main A* algorithm
    std::vector<GraphNode> getNeighbors();
};
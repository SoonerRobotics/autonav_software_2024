#pragma once

#include <math.h>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <fstream>
#include <unordered_map>

#include <filesystem>


#include "scr/node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/imu_data.hpp"
#include "autonav_msgs/msg/pathing_debug.hpp"
#include "autonav_msgs/msg/safety_lights.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "scr/json.hpp"


struct GraphNode {
    int x;
    int y;

    double g_cost;
    double h_cost;
    double f_cost;

    GraphNode* parent;

    // define the less_than operator so we can use std::sort on the list
    // https://stackoverflow.com/questions/1380463/sorting-a-vector-of-custom-objects
    bool operator < (const GraphNode& other) const {
        return (f_cost < other.f_cost);
    }

    bool operator == (const GraphNode& other) const {
        return (x == other.x && y == other.y);
    }

    std::string to_string() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }
};

struct GPSPoint {
    double lat;
    double lon;
};

struct AStarConfig {
    double waypointPopDistance;
    double waypointDirection;
    bool useOnlyWaypoints;
    double waypointDelay;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(AStarConfig, waypointPopDistance, waypointDirection, useOnlyWaypoints, waypointDelay)
};


class AStarNode : public SCR::Node {
public:
    AStarNode() : SCR::Node("astar_fast") {}
    ~AStarNode() {};

    void init() override;
    void system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override;
    void config_updated(nlohmann::json newConfig) override;
    nlohmann::json get_default_config() override;
private:
    // === ros things ===
    // subcribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr expandedSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr poseSubscriber;
    rclcpp::Subscription<autonav_msgs::msg::IMUData>::SharedPtr imuSubscriber;

    // message containers
    geometry_msgs::msg::Pose position;
    GPSPoint gps_position; // for smellification algorithm
    autonav_msgs::msg::IMUData imu;
    nav_msgs::msg::OccupancyGrid grid;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
    rclcpp::Publisher<autonav_msgs::msg::SafetyLights>::SharedPtr safetyPublisher;
    rclcpp::Publisher<autonav_msgs::msg::PathingDebug>::SharedPtr debugPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pathDebugImagePublisher;
    //TODO callback timers
    //TODO publisher methods or something


    // callbacks
    void onConfigSpaceReceived(nav_msgs::msg::OccupancyGrid grid_msg);
    
    void onPoseReceived(geometry_msgs::msg::Pose pos_msg);
    void onImuReceived(autonav_msgs::msg::IMUData imu_msg);

    void OnReset();

    //TODO publisher callback/timer/whatnot

    // === /end ros things ===



    // === a* things ===
    // main actual search function
    std::vector<GraphNode> Search(GraphNode start, GraphNode goal);

    // helper function to get the minecraft crafting table neighbors
    std::vector<GraphNode> GetNeighbors(GraphNode node, bool canGoBackwards = true);

    // traverse the nodes backwards to the start and return that
    std::vector<GraphNode> ReconstructPath(GraphNode goal);

    // helper function to update a node's information
    void UpdateNode(GraphNode node, double g_cost, double h_cost, GraphNode current);

    // our heuristic (h_cost), literally just distance formula
    double DistanceFormula(GraphNode current, GraphNode goal);

    // big function to call all the other functions
    void DoAStar();

    // convert the path to ROS Poses
    nav_msgs::msg::Path ToPath(std::vector<GraphNode> nodes);

    // get out goal node using the smelly algorithm (bias against obstacles and lanes, bias towards the middle)
    GraphNode Smellification();

    // get us our waypoints
    std::vector<GPSPoint> GetWaypoints();

    // return a safety lights message from RGB arguments
    autonav_msgs::msg::SafetyLights GetSafetyLightsMsg(int red, int green, int blue);

    // get distance between two GPS points (using custom in-house formula that I don't know what it does)
    double GpsDistanceFormula(GPSPoint goal, GPSPoint currPose);

    // get difference between two angles
    double GetAngleDifference(double angle1, double angle2); //TODO write

    // ======== fields ========

    // open list
    std::vector<GraphNode> frontier;

    // have been searched list
    std::vector<GraphNode> closed;
    
    // full map grid thingamajig (0 is traversable/open, 1 is an obstacle)
    std::vector<std::vector<int>> map;

    size_t count_;
    int MAX_X = 80; //TODO assign these values (or figure out how to calculate them automagically)
    int MAX_Y = 80;
    GraphNode start_node; //TODO initialize this one with wherever the heck we're starting at

    // filename for the waypoints (should be CSV file with label,lat,lon)
    const std::string WAYPOINTS_FILENAME = "./data/waypoints.csv";
    std::ifstream waypointsFile;
    std::unordered_map<std::string, std::vector<GPSPoint>> waypointsDict; // dictionairy of a list of double tuples

    // smelly bits
    std::vector<GraphNode> smellyFrontier; // store our frontier for breadth-first searching for a goal node
    std::vector<GraphNode> smellyExplored; // store our explored for breadth-first searching for a goal node
    std::vector<GPSPoint> waypoints; // gps waypoints we PID to //TODO this needs to be how it is and the other thing needs to be renamed
    double waypointTime = -1; // time we hit a waypoint or something
    double resetWhen = -1; // when to reset safety lights color
    double WAYPOINT_POP_DISTANCE = 1.1; //FIXME tune this is from last year
    double WAYPOINT_DELAY = 10; //TODO this needs to be configurable or something

    double MAX_DEPTH = 50; // max depth for the breadth-first search
    double SMELLY_Y = 80; //TODO this might need to be MAX_Y or something
    double SMELLY_Y_COST = 1.3; //FIXME this is from last year
    double SMELLY_DEPTH_COST = 2.2; //FIXME this is from last year
    double LATITUDE_LENGTH = 111086.2;
    double LONGITUDE_LENGTH = 81978.2;

    AStarConfig config;

    // === /a* things ===
};
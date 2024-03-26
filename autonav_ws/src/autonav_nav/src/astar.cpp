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
#include "autonav_msgs/msg/gps_feedback.hpp"

// JSON stuff (for config)
#include "scr/json.hpp"
using json = nlohmann::json;


// === structs ===
// struct to represent a node on the graph (essentially a square in the grid/map, but like math-technically it's a 'node' or something)
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

// pretty basic gps point struct
struct GPSPoint {
    double lat;
    double lon;
};


class AStarNode : public SCR::Node {
public:
    // we don't use constructors here
    AStarNode() : SCR::Node("astar_fast")  {};
    ~AStarNode() {};


    // initialization method (because we have no constructors) also reads GPS waypoints from a file
    void init() {
        // initialize our starting position
        position.lat = 0;
        position.lon = 0;

        // === read waypoints from file ===
        std::string line;
        waypointsFile.open(WAYPOINTS_FILENAME);
        getline(waypointsFile, line); // skip the first line
        while (getline(waypointsFile, line) ) {
            std::vector<std::string> tokens; // https://www.geeksforgeeks.org/tokenizing-a-string-cpp/
            std::stringstream strstream(line);
            std::string intermediate;
            while(getline(strstream, intermediate, ',')) {
                tokens.push_back(intermediate);
            }

            GPSPoint point;
            point.lat = std::stod(tokens[1]); //https://cplusplus.com/reference/string/stod/
            point.lon = std::stod(tokens[2]);

            // waypoints are stored like {"north":[GPSPoint, GPSPoint]}
            waypointsDict[tokens[0]].push_back(point);
        }
        waypointsFile.close();
        // === /read waypoints ===


        // subscribers and publisher
        expandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20, std::bind(&onOccupancyGridReceived, this, std::placeholders::_1));
        poseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("/autonav/position", 20, std::bind(&onPositionReceived, this, std::placeholders::_1));
        pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/autonav/path", 20);

        // we've set everything up so now we're operating
        set_device_state(SCR::DeviceState::OPERATING);
    }

    // callback for GPS position
    void onPositionReceived(geometry_msgs::msg::Pose msg) {
        this->position.lat = msg.position.y;
        this->position.lon = msg.position.x;

        //TODO we probably want to do something with the heading or something
    }

    // callback for occupancy grid
    void onOccupancyGridReceived(nav_msgs::msg::OccupancyGrid msg) {
        this->map = msg.data; // take the data and run with it

        // and then publish our path
        this->publishPath();
    }

    // convert what's returned by A* into a publishable path and publish it
    void publishPath() {
        // get the path to the goal
        std::vector<GraphNode> path = this->findPath();

        // if A* didn't find anything
        if (path.size() == 0) {
            //TODO don't we just like not publish anything then? or do we need to like tell somebody, because theoretically we should always be able to find a path right?
        } else { // otherwise we found something and need to publish it
            // message we're publishing
            nav_msgs::msg::Path pathMsg;

            // chuck the header into it
            pathMsg.header = std_msgs::msg::Header();
            pathMsg.header.stamp = this->now();

            // for each node in the path
            for(GraphNode node : path) {
                // we need to append a Pose for it, which requires a few other messages to add to it
                geometry_msgs::msg::PoseStamped poseStamped;
                geometry_msgs::msg::Pose pose;
                geometry_msgs::msg::Point point;

                //FIXME this should be giving the lat and lon, no?
                point.x = node.x;
                point.y = node.y;

                // reassemble/constructify the PoseStamped message
                pose.position = point;
                poseStamped.pose = pose;

                // add the position to the actual message
                pathMsg.poses.push_back(poseStamped);
            }

            this->pathPublisher->publish(pathMsg);
        }
    }

    std::vector<GraphNode> findPath() {
        std::vector<GraphNode> pathSoFar;

        // empty the frontier
        this->frontier.erase();

        //TODO make startNode variable thingamajig

        // push the starting node for our search onto the frontier
        this->frontier.push(startNode);

        // combined smellification algorithm
        int depth = 0;
        GraphNode best;
        best.f_cost = 9999;

        GraphNode current;
        current.f_cost = 9999;

        // begin the search
        while (depth < MAX_DEPTH) {
            // for each node on our frontier
            for (GraphNode node : frontier) {
                //TODO calculate costs
                //TODO update costs

                // for each neighbor of our current node
                for (GraphNode neighbor : getNeighbors(node)) {
                    // if the neighbor is already in the to-be-explored list
                    if (neighbor in smellyFrontier) {
                        continue; // skip it
                    } 

                    // otherwise, it should be in the frontier
                    smellyFrontier.push_back(neighbor);
                }

                // if the current node is cheaper to go to
                if (node.f_cost < best.f_cost) {
                    // then replace the current best with the new best
                    best.x = node.x;
                    best.y = node.y;
                    best.f_cost = node.f_cost;
                }

                // once we've reached here we've obviously explored it, so tack it to the explored list
                smellyExplored.push_back(node);
            }

            depth++;
        }

        //TODO make path from best cost back to start
    }

private:
    // Y and X dimensions for the occpancy grid
    const static int MAX_Y = 80;
    const static int MAX_X = 80;

    // member fields
    //https://www.geeksforgeeks.org/priority-queue-in-cpp-stl/ and  https://cplusplus.com/reference/queue/priority_queue/
    std::priority_queue<GraphNode> frontier; // priority queue (aka heap, heapqueue, etc) of all the points we need to explore next for A* (priority queue is used because it is fast and good)
    std::vector<GraphNode> closed; // nodes we have explored
    std::vector<signed char, std::allocator<signed char>> map; // local map data
    GPSPoint position; // position of robot (lat, lon)
    std::vector<GPSPoint> waypoints; // gps waypoints we're PIDing to

    bool getNewGpsCoords; // whether we need to update the waypoints array or not
    const int MAX_DEPTH = 50; // max depth for A* / breadth-first-search / whatever kind of combined smellification algorithm we're doing

    // stuff for file-reading code
    const std::string WAYPOINTS_FILENAME = "./data/waypoints.csv"; // filename for the waypoints (should be CSV file with label,lat,lon,)
    std::ifstream waypointsFile; // actual C++ file object
    std::unordered_map<std::string, std::vector<GPSPoint>> waypointsDict; // dictionairy of lists containing the GPS waypoints we could PID to, choose the waypoints for the correct direction from here

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
    std::vector<GraphNode> getNeighbors(GraphNode node);
    std::vector<GraphNode> findPath();
};


// main method (don't put in separate file else it no worky)
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}

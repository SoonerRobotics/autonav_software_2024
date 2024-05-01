// C++ includes
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include <math.h>

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

#define INF_COST 9999 // to represent a node with infinite cost (ie unexplored kind of)

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
        expandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20, std::bind(&AStarNode::onOccupancyGridReceived, this, std::placeholders::_1));
        poseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("/autonav/position", 20, std::bind(&AStarNode::onPositionReceived, this, std::placeholders::_1));
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

    // get the city-block neighbors (except not backwards because we don't do that here)
    std::vector<GraphNode> getNeighbors(GraphNode node) {
        // get the two neighbors left and right
        // get the neighbor forwards (up)
        const int directions[4][2] = {{-1, 0}, {0, -1}, {0, 1}};

        std::vector<GraphNode> ret = {};

        // for each direction
        for (int i = 0; i < 4; i++) {
            // calculate the neighbor's x and y
            int x = node.x + directions[i][0];
            int y = node.y + directions[i][1];

            // if the x coordinate is in bounds
            if (0 < x && x < MAX_X) {
                // and the y coord is in bounds
                if (0 < y && y < MAX_Y) {
                    // and it's not an obstacle
                    if (map[MAX_X * y  +  x] != 0) { // row major order or something
                        // then it's a good neighbor
                        GraphNode neighbor;
                        neighbor.x = x;
                        neighbor.y = y;
                        neighbor.g_cost = INF_COST;
                        neighbor.h_cost = INF_COST;
                        neighbor.f_cost = INF_COST;

                        ret.push_back(neighbor);
                    }
                }
            }
        }

        return ret;
    }

    std::vector<GraphNode> findPath() {
        // create a new, empty frontier
        this->frontier = std::vector<GraphNode>(); //FIXME this needs to be a priorityqueue or something at some point yada yada linked list yada yada how many times has the type of this changed?
        this->frontier.resize((MAX_X * MAX_Y) / 2); // reserve our initial space

        // make the start node
        GraphNode startNode;
        startNode.x = MAX_X/2;
        startNode.y = MAX_Y;
        startNode.g_cost = 1; // initial cost to move here is 1
        startNode.h_cost = INF_COST;
        startNode.f_cost = INF_COST;

        // push the starting node for our search onto the frontier
        this->frontier.push_back(startNode);

        // combined smellification algorithm
        int depth = 0;
        GraphNode best;
        best.f_cost = INF_COST;

        GraphNode current;
        current.f_cost = INF_COST;

        GraphNode node;

        // begin the search
        while (depth < MAX_DEPTH) {
            // for each node on our frontier
            while(frontier.size() > 0) {
                node = frontier[0]; // the node we look at first is the one with the least cost
                frontier.erase(frontier.begin()); // remove it from the frontier (https://stackoverflow.com/questions/875103/how-do-i-erase-an-element-from-stdvector-by-index)

                // for each neighbor of the node
                for (GraphNode neighbor : getNeighbors(node)) {
                    // if the neighbor is already in the to-be-explored list
                    //FIXME maybe this also needs to check if it's already in the closed list???
                    if (std::find(closed.begin(), closed.end(), neighbor) != closed.end()) { //https://stackoverflow.com/questions/571394/how-to-find-out-if-an-item-is-present-in-a-stdvector
                        continue; // skip it
                    }

                    // calculate smellification costs
                    double x_cost = pow((neighbor.x - (MAX_X)/2), 2); // cost for x coordinate is (x - MAX/2)**2
                    double y_cost = -neighbor.y + MAX_Y; // cost for y coordinate is just linear (keep in mind that the top of the grid is y=0 and we are at y=MAX_Y)

                    // update A* costs
                    double g_cost = node.g_cost + 1; // cost to move there is current+1
                    //TODO should this be x_cost and y_cost?
                    double h_cost = dist(neighbor, startNode); // h_cost is just distance from here to the goal except we don't know where the goal is we're figuring that out as we go FIXME
                    h_cost = x_cost + y_cost; //FIXME

                    // otherwise, update it
                    neighbor.parent = &node; // neighbor should point to current node for it's parent
                    neighbor.g_cost = g_cost; // and update it with all the values we calculated
                    neighbor.h_cost = h_cost;
                    neighbor.f_cost = g_cost + h_cost;

                    // and add it to the frontier
                    this->frontier.push_back(neighbor);
                }

                // if the current node is cheaper to go to
                if (node.f_cost < best.f_cost) {
                    // then replace the current best with the new best
                    best.x = node.x;
                    best.y = node.y;
                    best.f_cost = node.f_cost;
                }

                // once we've reached here we've obviously explored it, so tack it to the explored list
                closed.push_back(node);
            }

            depth++;
        }

        // once we're reached here, we should have a path somewhere, so find it
        std::vector<GraphNode> path = std::vector<GraphNode>();
        current = best;
        while (true) {
            // get the parent 
            node = *current.parent;

            // and add it to the path
            path.push_back(node);

            // if we've reached the start node, then we've found the path
            if (node == startNode) {
                return path;
            }

            // and move our current back to the node
            current = node;
        }

        // sort the frontier so lowest-cost is first and gets searched first
        std::sort(frontier.begin(), frontier.end()); //https://stackoverflow.com/questions/2758080/how-to-sort-an-stl-vector
    }

    // return the distance between two graphNodes
    double dist(GraphNode here, GraphNode there) {
        return sqrt((here.x - there.x)*(here.x-there.x)  +  (here.y-there.y)*(here.y-there.y));
    }

private:
    // Y and X dimensions for the occpancy grid
    const static int MAX_Y = 80;
    const static int MAX_X = 80;

    // member fields
    //https://www.geeksforgeeks.org/priority-queue-in-cpp-stl/ and  https://cplusplus.com/reference/queue/priority_queue/
    std::vector<GraphNode> frontier; // list of all the points we need to explore next for A* (priority queue would be used because it is fast and good but we didn't)
    std::vector<GraphNode> closed; // nodes we have explored
    std::vector<int> map; // to hold the map data from the OccupancyGrid msg
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
    
    // publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
};


// main method (don't put in separate file else it no worky)
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}

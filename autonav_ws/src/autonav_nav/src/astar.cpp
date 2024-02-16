#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"
#include "autonav_nav/astar.h"



void AStarNode::init() {
    //TODO
    // std::vector<std::vector<GraphNode>> map;
    //TODO make the file reading code so we can a list of waypoints and stuff
    //TODO make smellification or whatever goal-finding heuristic algorithm thingy
    //TODO write actual subscriber methods and whatnot

    frontier.reserve(100);
    closed.reserve(100);


    // left/right filtered subscribers
    leftExpandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded/left", 20, std::bind(&AStarNode::onLeftReceived, this, std::placeholders::_1));
    rightExpandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded/right", 20, std::bind(&AStarNode::onRightReceived, this, std::placeholders::_1));
    
    // localization data subscribers
    poseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("/autonav/position", 20, std::bind(&AStarNode::onPoseReceived, this, std::placeholders::_1));
    imuSubscriber = this->create_subscription<autonav_msgs::msg::IMUData>("/autonav/imu", 20, std::bind(&AStarNode::onImuReceived, this, std::placeholders::_1));
    // raw_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>(directionify("/autonav/cfg_space/raw"), 20, std::bind(&ExpandifyNode::onConfigSpaceReceived, this, std::placeholders::_1));

    // === publishers ===
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/autonav/path", 20);
    //TODO path publisher, safety lights publisher, debug publisher, debug image publisher

    //TODO callback timer

    // we've set everything up so now we're operating
    set_device_state(SCR::DeviceState::OPERATING);
}

void AStarNode::onLeftReceived(nav_msgs::msg::OccupancyGrid grid_msg) {
    numLeft++;

    leftGrid = grid_msg;
}

void AStarNode::onRightReceived(nav_msgs::msg::OccupancyGrid grid_msg) {
    numRight++;

    rightGrid = grid_msg;

    if (numRight == numLeft) {
        this->DoAStar();
    }
}

void AStarNode::onPoseReceived(geometry_msgs::msg::Pose pos_msg) {
    // copy the data over or something idk
    this->position = pos_msg;
}

//TODO figure out some kinda use for this?
void AStarNode::onImuReceived(autonav_msgs::msg::IMUData imu_msg) {
    this->imu = imu_msg;
}


void AStarNode::DoAStar() {
    // define our goal node
    //TODO do smellification and make sure our goal is reachable
    GraphNode goal;
    goal.x = MAX_X/2;
    goal.y = 0; // just go straight down the middle for now pretty much

    // make the starting node
    GraphNode start;
    start.x = MAX_X/2; // start in the middle
    start.y = MAX_Y; // start at the bottom, because arrays and indexing
    start.g_cost = 0;
    start.h_cost = DistanceFormula(start, goal);
    start.f_cost = start.g_cost + start.h_cost;

    // perform A*
    auto path = AStarNode::Search(start, goal);

    // publish results
    this->pathPublisher.publish(path);
}



// handy references:
//https://en.cppreference.com/w/cpp/container/vector
//https://en.cppreference.com/w/cpp/language/pointer
//https://llego.dev/posts/implementing-the-a-search-algorithm-python/
//https://cplusplus.com/reference/cmath/
//https://stackoverflow.com/questions/8877448/how-do-i-reverse-a-c-vector

// main actual search function (based off of https://llego.dev/posts/implementing-the-a-search-algorithm-python/ except the python code is wrong so it's only based off of)
std::vector<GraphNode> AStarNode::Search(GraphNode start, GraphNode goal) {
    GraphNode currentNode;
    frontier.push_back(start); // the start node will be the first to be expanded

    // while there are still nodes in the open list (frontier)
    while (frontier.size() > 0) {
        // sort the list to get the node with the lowest f_cost first
        std::sort(frontier.begin(), frontier.end());

        // get the first node (the one with the lowest cost)
        currentNode = frontier.at(0);
        frontier.erase(frontier.begin()); // screw C++ https://stackoverflow.com/questions/875103/how-do-i-erase-an-element-from-stdvector-by-index

        // add the current node to the list of closed nodes
        closed.push_back(currentNode);

        // if the curent node is the goal node, then we've reached our goal
        if (currentNode.x == goal.x && currentNode.y == goal.y) {
            // return the finished path so we can traverse it
            return ReconstructPath(goal);
        }

        // otherwise, get all the neighboring nodes
        std::vector<GraphNode> neighbors = GetNeighbors(currentNode);

        // for each neighbor
        for (GraphNode node : neighbors) {
            // if it's been visited (part of the closed list)
            // https://stackoverflow.com/questions/571394/how-to-find-out-if-an-item-is-present-in-a-stdvector
            if (std::find(closed.begin(), closed.end(), node) != closed.begin()) {
                // then skip
                continue;
            }

            // otherwise, calculate the costs
            double g_cost = currentNode.g_cost + 1; // cost to move there is 1 more than the current node we're checking the neighbors of
            double h_cost = DistanceFormula(node, goal); // heuristic is how far from the goal it is
            double f_cost = g_cost + h_cost;

            // if the neighbor we're checking is in the open list
            if (std::find(frontier.begin(), frontier.end(), node) != frontier.begin()) {
                // and if our new f_cost is less than its old f_cost
                if (f_cost < node.f_cost) {
                    // then update it
                    UpdateNode(node, g_cost, h_cost, currentNode);
                }
            // otherwise,
            } else {
                // update it because it hasn't been checked yet
                UpdateNode(node, g_cost, h_cost, currentNode);

                // and add it to the list of open nodes
                frontier.push_back(node);
            }
        }
    }

    // if we've made it this far without finding a path, there is no path, so return null or something idk
    std::vector<GraphNode> ret;
    return ret;
}


// helper function to get the minecraft crafting table neighbors
std::vector<GraphNode> AStarNode::GetNeighbors(GraphNode node) {
    std::vector<GraphNode> neighbors;

    // neighbors are just the surrounding nodes
    for (int x = -1; x < 1; x++) {
        for (int y = -1; y < 1; y++) {
            int neighbor_x = node.x + x;
            int neighbor_y = node.y + y;

            // check if it's in bounds
            if (0 < neighbor_x && neighbor_x < MAX_X) {
                if (0 < neighbor_y && neighbor_y < MAX_Y) {

                    // check if it's traversable (ie not an obstacle, so not == 1)
                    if (map[neighbor_x][neighbor_y] == 0) {
                        GraphNode node; //TODO this doesn't work I don't think
                        node.x = neighbor_x;
                        node.y = neighbor_y;

                        neighbors.push_back(node); // add it to the list
                    }
                }
            }
        }
    }

    return neighbors;
}


// standard distance formula (cartesian) between two nodse
double AStarNode::DistanceFormula(GraphNode current, GraphNode goal) {
    return sqrt(pow((current.x - goal.x), 2) + pow((current.y - goal.y), 2));
}


std::vector<GraphNode> AStarNode::ReconstructPath(GraphNode goal) {
    std::vector<GraphNode> path = {goal}; // the path starts at the goal node
    GraphNode current = goal; // and so we start at the goal node

    // while we haven't reached the start
    while (current.parent != &start_node) {
        // add the parent of the current node to the list
        path.push_back(*current.parent);

        // then move to the parent
        current = *current.parent;
    }

    // reverse the path (so that it's start to finish instead of finish to start)
    std::reverse(path.begin(), path.end()); //https://stackoverflow.com/questions/8877448/how-do-i-reverse-a-c-vector

    return path;
}


// update a node with new information
void AStarNode::UpdateNode(GraphNode node, double g_cost, double h_cost, GraphNode current) {
    node.g_cost = g_cost;
    node.h_cost = h_cost;
    node.f_cost = g_cost + h_cost;

    node.parent = &current;
}
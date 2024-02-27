#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"
#include "autonav_nav/astar.h"

#define PI 3.1415926535897932384626433


void AStarNode::init() {
    //TODO
    // std::vector<std::vector<GraphNode>> map;
    //TODO make the file reading code so we can a list of waypoints and stuff
    //TODO make smellification or whatever goal-finding heuristic algorithm thingy
    //TODO safety ligths publisher

    frontier.reserve(100);
    closed.reserve(100);


    // filtered subscriber
    expandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded/", 20, std::bind(&AStarNode::onConfigSpaceReceived, this, std::placeholders::_1));
    
    // localization data subscribers
    poseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("/autonav/position", 20, std::bind(&AStarNode::onPoseReceived, this, std::placeholders::_1));
    imuSubscriber = this->create_subscription<autonav_msgs::msg::IMUData>("/autonav/imu", 20, std::bind(&AStarNode::onImuReceived, this, std::placeholders::_1));
    // raw_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>(directionify("/autonav/cfg_space/raw"), 20, std::bind(&ExpandifyNode::onConfigSpaceReceived, this, std::placeholders::_1));

    // === publishers ===
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/autonav/path", 20);
    safetyPublisher = this->create_publisher<autonav_msgs::msg::SafetyLights>("/autonav/SafetyLights", 20);
    debugPublisher = this->create_publisher<autonav_msgs::msg::PathingDebug>("/autonav/debug/astar", 20);
    pathDebugImagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/debug/astar/image", 20);

    // we've set everything up so now we're operating
    set_device_state(SCR::DeviceState::OPERATING);
}


void AStarNode::onConfigSpaceReceived(nav_msgs::msg::OccupancyGrid grid_msg) {
    grid = grid_msg;

    this->DoAStar();
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
    GraphNode goal = this->Smellification();

    // make the starting node
    GraphNode start;
    start.x = MAX_X/2; // start in the middle
    start.y = MAX_Y; // start at the bottom, because arrays and indexing
    start.g_cost = 0;
    start.h_cost = DistanceFormula(start, goal);
    start.f_cost = start.g_cost + start.h_cost;

    // update the map with the grid data
    for (int x = 0; x < MAX_X; x++) {
        for (int y = 0; y < MAX_Y; y++) {
            // 1D array
            map[y][x] = grid.data[y+x];
        }
    }

    // perform A*
    auto pathMsg = AStarNode::ToPath(AStarNode::Search(start, goal));

    // only publish if we found a path
    if (pathMsg.poses.size() > 0) {
        // publish results
        this->pathPublisher->publish(pathMsg);

        std::size_t totalsize = 0;
        for (int x = 0; x < map.size(); x++) {
            totalsize += map[x].size();
        }

        auto data = new int[totalsize]();
        for (int y = 0; y < map.size(); y++) {
            for (int x = 0; x < map[y].size(); x++) {
                data[y*MAX_X + x] = map[y][x];
            }
        }

        // draw the cost map onto a debug image
        auto image = cv::Mat(MAX_X, MAX_Y, CV_8UC1, data); //TODO define data and double check size

        // fill it with 0s to start
        // std::fill(cvimg.begin(), cvimg.end()), 0;

        for (int x = 0; x < MAX_X; x++) {
            for (int y = 0; y < MAX_Y; y++) {
                // at the pixel at (x, y), draw the presence of an obstacle or not (*255 will make 1 into pure white)
                image[x][y] = map[x][y] * 255;
            }
        }
        /**
        for pp in path: //TODO draw the points along the path in blue
            cv2.circle(cvimg, (pp[0], pp[1]), 1, (0, 255, 0), 1)
        */

        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        cv::resize(image, image, cv::Size(MAX_X, MAX_Y), 0, 0, cv::INTER_NEAREST);

        auto compressed = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
        compressed->header.stamp = this->now();
        compressed->header.frame_id = "map";
        compressed->format = "jpeg";
        
        this->pathDebugImagePublisher->publish(*compressed);

        delete[] data;
    }
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
        //TODO priority queue sorted linked list according to Noah
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
            // don't return ourself
            if (x == 0 && y == 0) {
                continue;
            }

            int neighbor_x = node.x + x;
            int neighbor_y = node.y + y;

            // check if it's in bounds
            if (0 < neighbor_x && neighbor_x < MAX_X) {
                if (0 < neighbor_y && neighbor_y < MAX_Y) {

                    // check if it's traversable (ie not an obstacle, so not == 1)
                    if (map[neighbor_x][neighbor_y] == 0) {
                        GraphNode node;
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

nav_msgs::msg::Path AStarNode::ToPath(std::vector<GraphNode> nodes) {
    nav_msgs::msg::Path pathMsg;
    pathMsg.header = std_msgs::msg::Header(); //TODO add actual header data or something
    // pathMsg.poses = new int[(MAX_X * MAX_Y) / 2]{}; // theoretical max path length is the entire map, but half of that seems reasonably safe
    pathMsg.poses = std::vector<geometry_msgs::msg::PoseStamped>(); // theoretical max path length is the entire map, but half of that seems reasonably safe

    for (GraphNode node : nodes) {
        // and PoseStamped is just Header header, Pose pose
        // and Pose is just Point position, Quaternion orientation
        // and Point is just x, y, z; and Quaternion is just x, y, z, w

        // make the poseStamped
        auto poseStamped = geometry_msgs::msg::PoseStamped();

        // add the header
        poseStamped.header = std_msgs::msg::Header(); //TODO actual header data

        // make the Pose part of the PoseStamped
        geometry_msgs::msg::Pose pose;

        // make the Point part of the Pose
        geometry_msgs::msg::Point point;

        // the actual bit that's important
        point.x = node.x;
        point.y = node.y;

        // put things where they need to be
        pose.position = point;
        poseStamped.pose = pose;

        pathMsg.poses.push_back(poseStamped); //TODO I don't think you can do this
    }
}

// ======================================
std::vector<std::vector<double>> AStarNode::GetWaypoints() {
    return; //TODO write
}

// get a safety lights message from parameters
autonav_msgs::msg::SafetyLights AStarNode::GetSafetyLightsMsg(int red, int green, int blue) {
    autonav_msgs::msg::SafetyLights msg;

    // FIXME default config values used here, but at some point we might want to be able to change them
    msg.mode = 0;
    msg.autonomous = true;
    msg.eco = false;
    msg.brightness = 255;

    // actual important color bits
    msg.red = red;
    msg.green = green;
    msg.blue = blue;

    return msg;
}

// distance formula for GPS coordinates
double AStarNode::GpsDistanceFormula(std::vector<double> goal, geometry_msgs::msg::Pose currPosition) {
    return -1; //TODO write from Python
}

//TODO file code for CSV waypoints

// smelly algorithm (bias towards center, away from obstacles, away from lanes, towards goal heading) to identify a goal node
GraphNode AStarNode::Smellification() {
    int depth = 0;
    smellyFrontier = frontier;
    double cost = 0;
    double bestCost = 0;
    GraphNode bestPos;
    bestPos.x = (int)(MAX_X/2); //FIXME I have no idea what coordinates go where
    bestPos.y = 0; //??????
    // smellification idk

    // while we haven't hit the max depth and still have nodes to explore
    while (depth < MAX_DEPTH && smellyFrontier.size() > 0) {
        // we should explore those nodes
        for (int i = 0; i < smellyFrontier.size(); i++) {
            auto node = smellyFrontier[i];

            // cost is based on y coordinate and depth (favoring shorter paths)
            cost = (SMELLY_Y - node.y) * SMELLY_Y_COST  +  (depth * SMELLY_DEPTH_COST);

            // get the current time
            std::chrono::duration<double, std::chrono::seconds> seconds_ = std::chrono::system_clock::now(); // - 0
            double seconds = seconds_.count();

    
            //TODO document
            if (size(waypoints) == 0  &&  seconds > waypointTime  &&  waypointTime != 0) {
                this->waypoints = this->GetWaypoints();
                this->waypointTime = 0;
            //TODO document
            } else if (seconds < this->waypointTime  &&  this->waypoints.size() == 0) { 
                autonav_msgs::msg::PathingDebug pathingDebugMsg;
                pathingDebugMsg.waypoints = {};
                pathingDebugMsg.time_until_use_waypoints = this->waypointTime - seconds;
                this->debugPublisher->publish(pathingDebugMsg);
            }

            //TODO document and maybe move up?
            if (this->resetWhen != -1  &&  seconds > this->resetWhen  &&  this->mobility) {
                this->safetyPublisher->publish(this->GetSafetyLightsMsg(255, 255, 255));
                this->resetWhen = -1;
            }

            //TODO document and maybe make it a part of the if/else chain?
            if (waypoints.size() > 0) {
                auto next_waypoint = this->waypoints[0];
                auto heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi);

                if (GpsDistanceFormula(next_waypoint, this->position) <= this->waypointPopDistance) {
                    this->waypoints.pop(0);
                    this->safetyPublisher.publish(this->GetSafetyLightsMsg(0, 255, 0));
                    this->resetWhen = seconds + 1.5;
                }

                autonav_msgs::msg::PathingDebug debugMsg;
                debugMsg.desired_heading = heading_to_gps;
                debugMsg.desired_latitude = next_waypoint[0];
                debugMsg.desired_longitude = next_waypoint[1];
                debugMsg.distance_to_destination = GpsDistanceFormula(next_waypoint, this->position);

                // convert waypoints to 1d array
                auto waypoint1Darr = new int[this->waypoints.size()*2];
                int j = 0;
                for (int i = 0; i < this->waypoints.size(); i++) {
                    waypoint1Darr[j] = this->waypoints[i][0];
                    waypoint1Darr[j+1] = this->waypoints[i][1];
                    j += 2
                }
                debugMsg.waypoints = waypoint1Darr;
                this->debugPublisher->publish(debugMsg);
                delete waypoint1Darr;
            }
            //=======================================================

            if (this->waypoints.size() > 0) {
                //TODO what the heck do the 40 and 80 and what even is this doing hello figure this out please
                double heading_error = abs(getAngleDifference(this->heading + atan2(40 - node.x, 80 - node.y), heading_to_gps)) * 180 / PI;
                cost -= std::max(heading_error, (double)10);
            }

            // if the current cost is larger than our running best, then update that
            if (cost > bestCost) {
                bestCost = cost;
                bestPos = node;
            }

            //TODO fix this
            frontier.remove(pos);
            // explored.add(x + 80 * y)

            // if y > 1 and grid_data[x + 80 * (y-1)] < 50 and x + 80 * (y-1) not in explored:
            //     frontier.add((x, y - 1))

            // if x < 79 and grid_data[x + 1 + 80 * y] < 50 and x + 1 + 80 * y not in explored:
            //     frontier.add((x + 1, y))

            // if x > 0 and grid_data[x - 1 + 80 * y] < 50 and x - 1 + 80 * y not in explored:
            //     frontier.add((x - 1, y))
        }

        depth++;
    }

    return bestPos;
}
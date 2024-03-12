#include "autonav_nav/astar.h"

#define PI 3.1415926535897932384626433

#define NOW std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
#define PRINT_NODE(msg_, node_) RCLCPP_WARN(this->get_logger(), (std::string(msg_) + node_.to_string()).c_str());

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}


// main initilization method (because we don't really have a constructor for reasons)
void AStarNode::init() {
    RCLCPP_WARN(this->get_logger(), "INITIALIZATION STARTED");

    // initialize our arrays
    frontier = std::vector<GraphNode>();
    closed = std::vector<GraphNode>();
    map = std::vector<std::vector<int>>();
    
    // reserve space so we avoid constantly allocating and reallocating space
    frontier.reserve(100);
    closed.reserve(100);

    map.resize(100);
    for (int i = 0; i < 100; i++) {
        map[i] = std::vector<int>();
        map[i].resize(100);
    }

    // === subscribers ===
    // filtered subscriber
    expandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20, std::bind(&AStarNode::onConfigSpaceReceived, this, std::placeholders::_1));
    
    // localization data subscribers
    poseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("/autonav/position", 20, std::bind(&AStarNode::onPoseReceived, this, std::placeholders::_1));
    imuSubscriber = this->create_subscription<autonav_msgs::msg::IMUData>("/autonav/imu", 20, std::bind(&AStarNode::onImuReceived, this, std::placeholders::_1));
    RCLCPP_WARN(this->get_logger(), "SUBSCRIBERS DONE");

    // === publishers ===
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/autonav/path", 20);
    safetyPublisher = this->create_publisher<autonav_msgs::msg::SafetyLights>("/autonav/SafetyLights", 20);
    debugPublisher = this->create_publisher<autonav_msgs::msg::PathingDebug>("/autonav/debug/astar", 20);
    pathDebugImagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/debug/astar/image", 20);
    RCLCPP_WARN(this->get_logger(), "PUBLISHERS DONE");

    // === read waypoints from file ===
    waypointsFile.open(WAYPOINTS_FILENAME);
    int numWaypoints = 0;
    bool firstLine = false;

    // loop through the lines in the file
    std::string line;
    if (waypointsFile.is_open()) {
        RCLCPP_WARN(this->get_logger(), "YES FILE IS OPEN");
        while (getline(waypointsFile, line) ) {
            if (!firstLine) { // first line is the one with the labels, we need to skip it
                firstLine = true;
                continue;
            }

            // https://www.geeksforgeeks.org/tokenizing-a-string-cpp/
            std::vector<std::string> tokens;
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
            numWaypoints++;
        }
    }
    RCLCPP_WARN(this->get_logger(), "NUM WAYPOINTS: ");
    RCLCPP_WARN(this->get_logger(), std::to_string(numWaypoints).c_str());

    waypointsFile.close();
    RCLCPP_WARN(this->get_logger(), "WAYPOINTS READ");
    // === /read waypoints ===

    // go ahead and initialize our waypointTime
    this->waypointTime = NOW + this->WAYPOINT_DELAY;

    // we've set everything up so now we're operating
    set_device_state(SCR::DeviceState::OPERATING);
}

// system state callback function
void AStarNode::system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) {
    RCLCPP_WARN(this->get_logger(), "SYSTEM STATE TRANSITION");
    
    if (updated.state == SCR::SystemState::AUTONOMOUS && updated.mobility && this->waypoints.size() == 0) {
        // if we just changed to autonomous and we're now allowed to move and we don't have waypoints yet
        this->waypointTime = NOW + this->WAYPOINT_DELAY; // then set the delay for using the waypoints
    } else if (updated.state != SCR::SystemState::AUTONOMOUS and this->device_state == SCR::DeviceState::OPERATING) {
        // otherwise, if we're not changing to auto but we are operating (ie we're changing to disabled or something)
        this->OnReset(); // reset
    }
}

// reset function
void AStarNode::OnReset() {
    RCLCPP_WARN(this->get_logger(), "ON RESET");
    this->imu = autonav_msgs::msg::IMUData();
    this->position = geometry_msgs::msg::Pose();
    this->waypoints = std::vector<GPSPoint>();
    this->waypointTime = NOW + this->WAYPOINT_DELAY;
}

// config update callback
//TODO we don't actually use any of the config so fix
void AStarNode::config_updated(nlohmann::json newConfig) {
    RCLCPP_WARN(this->get_logger(), "ON CONFIG UPDATED");

    this->config = newConfig.template get<AStarConfig>();
}

// return default config for initializing the display.html data
//TODO document and fix this is awful why do we have these do we even want these
nlohmann::json AStarNode::get_default_config() {
    RCLCPP_WARN(this->get_logger(), "GET DEFAULT CONFIG");

    AStarConfig defaultConfig;

    defaultConfig.waypointPopDistance = 1.1;
    defaultConfig.waypointDirection = 0;
    defaultConfig.useOnlyWaypoints = false;
    defaultConfig.waypointDelay = 17.5;

    return defaultConfig;
}


// main callback
void AStarNode::onConfigSpaceReceived(nav_msgs::msg::OccupancyGrid grid_msg) {
    RCLCPP_WARN(this->get_logger(), "GOT CONFIG SPACE");

    grid = grid_msg;

    // perform A* every time we get new data, we don't need to otherwise
    this->DoAStar();
}

// gps data callback
void AStarNode::onPoseReceived(geometry_msgs::msg::Pose pos_msg) {
    RCLCPP_WARN(this->get_logger(), "GOT GPS");

    this->position = pos_msg;
}

// imu data callback
void AStarNode::onImuReceived(autonav_msgs::msg::IMUData imu_msg) {
    //TODO figure out some kinda use for this?
    this->imu = imu_msg;
}

// main function of the A* node but not really
void AStarNode::DoAStar() {
    RCLCPP_WARN(this->get_logger(), "STARTING A*");

    // make the starting node (our initial position on the relative map)
    GraphNode start;
    start.x = (int)((double)MAX_X/2); // start in the middle
    start.y = MAX_Y; // start at the bottom, because arrays and indexing
    start.g_cost = 0;
    // start.h_cost = DistanceFormula(start, goal);
    start.h_cost = 0;
    start.f_cost = start.g_cost + start.h_cost;

    PRINT_NODE("START NODE: ", start);

    RCLCPP_WARN(this->get_logger(), "UPDATING THE GRID DATA");
    RCLCPP_WARN(this->get_logger(), (std::string("GRID SIZE: ") + std::to_string(grid.data.size())).c_str());
    // update the map with the grid data
    for (int x = 0; x < MAX_X; x++) {
        for (int y = 0; y < MAX_Y; y++) {
            // RCLCPP_WARN(this->get_logger(), ("(" + std::to_string(x) + ", " + std::to_string(y) + ")").c_str());
            // RCLCPP_WARN(this->get_logger(), ("GRID DATA AT CELL: " + std::to_string(grid.data[y+x])).c_str());
            // RCLCPP_WARN(this->get_logger(), ("MAP DATA AT CELL: " + std::to_string(map[y][x])).c_str());

            // grid is a 1D array, map is a 2D array
            map[y][x] = grid.data[y+x];
        }
    }

    // initialize our frontier with our starting position so Smellification actually works
    frontier.push_back(start);

    // find our goal node using smellification algorithm
    GraphNode goal = this->Smellification();

    // perform A*
    auto pathMsg = AStarNode::ToPath(AStarNode::Search(start, goal));

    // only publish if we found a path
    if (pathMsg.poses.size() > 0) {
        RCLCPP_WARN(this->get_logger(), "PUBLISHING PATH");

        // publish results
        this->pathPublisher->publish(pathMsg);

        RCLCPP_WARN(this->get_logger(), "PUBLISHING IMAGE");


        // find the total size of the 2D array
        std::size_t totalsize = 0;
        for (int x = 0; x < (int)map.size(); x++) {
            totalsize += map[x].size();
        }

        // make a 1D array with that size (to make the CV Mat image)
        auto data = new int[totalsize]();
        for (int y = 0; y < (int)map.size(); y++) {
            for (int x = 0; x < (int)map[y].size(); x++) {
                // one dimensionalize the 2D array
                data[y*MAX_X + x] = map[y][x];
            }
        }

        RCLCPP_WARN(this->get_logger(), "MAKING DEBUG IMAGE");
        // draw the cost map onto a debug image
        auto image = cv::Mat(MAX_X, MAX_Y, CV_8UC1, data); //TODO define data and double check size

        // loop through the image
        for (int x = 0; x < MAX_X; x++) {
            for (int y = 0; y < MAX_Y; y++) {
                // at the pixel at (x, y), draw the presence of an obstacle or not (*255 will make 1 into pure white)
                // image[x][y] = map[x][y] * 255;
                // image.at<uchar>(y, x) = 128; // per https://docs.opencv.org/3.4/d5/d98/tutorial_mat_operations.html
                image.at<uchar>(y, x) = (unsinged char)(map[y][x]*50);
            }
        }

        // convert it from grayscale and resize it
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        cv::resize(image, image, cv::Size(MAX_X, MAX_Y), 0, 0, cv::INTER_NEAREST);

        // make the actual ROS image
        auto compressed = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
        compressed->header.stamp = this->now();
        compressed->header.frame_id = "map";
        compressed->format = "jpeg";

        RCLCPP_WARN(this->get_logger(), "PUBLISHING DEBUG IMAGE");
        // publish the A* output as an image
        this->pathDebugImagePublisher->publish(*compressed);

        // manual memory management because we have to use arrays for the message instead of vectors
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
        RCLCPP_WARN(this->get_logger(), "STARTING SEARCH");

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
            RCLCPP_WARN(this->get_logger(), "FOUND GOAL");

            // return the finished path so we can traverse it
            return ReconstructPath(goal);
        }

        // otherwise, get all the neighboring nodes
        std::vector<GraphNode> neighbors = GetNeighbors(currentNode);

        // RCLCPP_WARN(this->get_logger(), "SEARCHING NEIGHBORS");
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


// helper function to get the neighbors of the current node (excluding diagonals)
std::vector<GraphNode> AStarNode::GetNeighbors(GraphNode node, bool canGoBackwards) {
    int maxObstacle = 1;

    // PRINT_NODE("GETTING NEIGBORS OF: ", node);

    std::vector<GraphNode> neighbors;

    std::vector<std::vector<double>> addresses = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}}; // addresses of the 4 edge-adjacent neighbors
    if (!canGoBackwards) {
        addresses.erase(addresses.begin() + 3); // remove the 2nd index (ie x+0, y+1) so that we don't check behind us
        // this is used by the Smellification algorithm to avoid the nonsense that happens in Python

        //FIXME this is for smellification to work but it's bad
        maxObstacle = 50;
    }

    // loop through all the neighbors
    for(int i = 0; i < (int)addresses.size(); i++) {
        int neighbor_x = node.x + addresses[i][0];
        int neighbor_y = node.y + addresses[i][1];
        // RCLCPP_WARN(this->get_logger(), "LOOPING THROUGH NEIGHBOR ADDRESSES");

        // RCLCPP_WARN(this->get_logger(), std::string("(X, Y) => (" + std::to_string(neighbor_x) + ", " + std::to_string(neighbor_y) + ")").c_str());

        // check if it's in bounds
        if (0 <= neighbor_x && neighbor_x <= MAX_X) {
            // RCLCPP_WARN(this->get_logger(), "IN BOUNDS: X");
            if (0 <= neighbor_y && neighbor_y <= MAX_Y) {
                // RCLCPP_WARN(this->get_logger(), "IN BOUNDS: Y");

                // RCLCPP_WARN(this->get_logger(), ("OBSTACLE: " + std::to_string(map[neighbor_y][neighbor_x])).c_str());

                // check if it's traversable (ie not an obstacle, so != 1)
                if (map[neighbor_y][neighbor_x] < maxObstacle) {
                    // RCLCPP_WARN(this->get_logger(), "NOT AN OBSTACLE");
                    // make a new node to represent the neighbor
                    GraphNode node;
                    node.x = neighbor_x;
                    node.y = neighbor_y;

                    neighbors.push_back(node); // add it to the list
                }
            }
        }
    }

    // return the list
    return neighbors;
}


// standard distance formula (cartesian) between two nodse
double AStarNode::DistanceFormula(GraphNode current, GraphNode goal) {
    return sqrt(pow((current.x - goal.x), 2) + pow((current.y - goal.y), 2));
}


// function to get the path from the goal to the start (traverses the pointers of each GraphNode)
std::vector<GraphNode> AStarNode::ReconstructPath(GraphNode goal) {
    RCLCPP_WARN(this->get_logger(), "RECONSTRUCTING PATH");

    std::vector<GraphNode> path = {goal}; // the path starts at the goal node
    GraphNode current = goal; // and so we start at the goal node

    RCLCPP_WARN(this->get_logger(), "RECONSTRUCT INITIALIZED");

    // while we haven't reached the start
    while (current.parent != &start_node) {
        // add the parent of the current node to the list
        path.push_back(*current.parent);

        // then move to the parent
        current = *current.parent;
        RCLCPP_WARN(this->get_logger(), "TRAVERSING...");
    }

    RCLCPP_WARN(this->get_logger(), "REVERSING...");
    // reverse the path (so that it's start to finish instead of finish to start)
    std::reverse(path.begin(), path.end()); //https://stackoverflow.com/questions/8877448/how-do-i-reverse-a-c-vector

    RCLCPP_WARN(this->get_logger(), "GOT PATH");

    return path;
}


// update a node with new information
void AStarNode::UpdateNode(GraphNode node, double g_cost, double h_cost, GraphNode current) {
    RCLCPP_WARN(this->get_logger(), "UPDATING NODE");

    // pretty standard stuff
    node.g_cost = g_cost;
    node.h_cost = h_cost;
    node.f_cost = g_cost + h_cost;

    // parent is a pointer to a GraphNode that is the parent of this node
    node.parent = &current;
}

// convert a list of nodes to a ROS Path message
nav_msgs::msg::Path AStarNode::ToPath(std::vector<GraphNode> nodes) {
    RCLCPP_WARN(this->get_logger(), "CONVERTING TO PATH MSG");

    nav_msgs::msg::Path pathMsg;
    pathMsg.header = std_msgs::msg::Header(); //TODO add actual header data or something

    // default size of 100 ought to be sufficient for a 100x100 grid (rarely do we do more than go straight)
    pathMsg.poses = std::vector<geometry_msgs::msg::PoseStamped>(100); // wait this is a vector that's so sick

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

        pathMsg.poses.push_back(poseStamped);
    }

    RCLCPP_WARN(this->get_logger(), "MADE PATH MSG");

    return pathMsg;
}

// ======================================
// get a safety lights message from parameters
autonav_msgs::msg::SafetyLights AStarNode::GetSafetyLightsMsg(int red, int green, int blue) {
    RCLCPP_WARN(this->get_logger(), "TRANSLATING TO SAFETY MSG");

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

// get the waypoints vector list thingy from the waypoints dictionary that stores all the waypoints we read from the file
std::vector<GPSPoint> AStarNode::GetWaypoints() {
    RCLCPP_WARN(this->get_logger(), "GETTING WAYPOINTS VECTOR");

    /**
    var yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion answer #2
    */
    // Get our current heading and estimate within 180 degrees which direction we are facing (north or south, 0 and 1 respectively)
    //NOTE this code is copied from the python A* file
    // auto heading_degrees = abs(this->position.orientation * 180 / PI);
    auto q = this->position.orientation;
    auto heading_degrees = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    bool north = false;
    
    // if we're mostly facing towards 180 degrees, then it's probably North
    if (120 < heading_degrees && heading_degrees < 240) {
        north = true;
        RCLCPP_WARN(this->get_logger(), "NORTH");
    } else {
        north = false;
        RCLCPP_WARN(this->get_logger(), "SOUTH");
    }

    RCLCPP_WARN(this->get_logger(), "System mode: ");
    RCLCPP_WARN(this->get_logger(), std::to_string(this->system_mode).c_str());

    if (this->system_mode == SCR::SystemMode::SIMULATION) {
        // return simulation waypoints if we're in simulation
        RCLCPP_WARN(this->get_logger(), "WAYPOINTS SIZE:");
        RCLCPP_WARN(this->get_logger(), std::to_string(this->waypointsDict["simulation1"].size()).c_str());

        return this->waypointsDict["simulation1"];
    } else if (this->system_mode == SCR::SystemMode::COMPETITION) {
        // otherwise, if we're at competition, and we're facing north
        if (north) {
            RCLCPP_WARN(this->get_logger(), "WAYPOINTS SIZE:");
            RCLCPP_WARN(this->get_logger(), std::to_string(this->waypointsDict["compNorth"].size()).c_str());

            // return the waypoints list for north
            return this->waypointsDict["compNorth"];
        } else {
            RCLCPP_WARN(this->get_logger(), "WAYPOINTS SIZE:");
            RCLCPP_WARN(this->get_logger(), std::to_string(this->waypointsDict["compSouth"].size()).c_str());

            // otherwise south
            return this->waypointsDict["compSouth"];
        }
    } else {
        return this->waypointsDict["practice"];
    }
}

// get distance from GPS coordinates
double AStarNode::GpsDistanceFormula(GPSPoint goal, GPSPoint currPose) {
    // calculate lat and lon offsets
    auto north_to_gps = (goal.lat - currPose.lat) * this->LATITUDE_LENGTH;
    auto west_to_gps = (currPose.lon - goal.lon) * this->LONGITUDE_LENGTH;

    // square each and add them together
    return (north_to_gps*north_to_gps) + (west_to_gps*west_to_gps);
}

// get the difference between two angles but like wrapped to 2PI
double AStarNode::GetAngleDifference(double angle1, double angle2) {
    // FIXME literally just stolen from the python code I don't know what this does
    auto delta = angle1 - angle2;
    delta = fmod((delta + PI), (2 * PI) - PI);

    return delta;
}


// smelly algorithm (bias towards center, away from obstacles, away from lanes, towards goal heading) to identify a goal node
GraphNode AStarNode::Smellification() {
    RCLCPP_WARN(this->get_logger(), "PERFORMING SMELLIFICATION");

    smellyFrontier = frontier; //FIXME I don't think copying over the frontier is right
    this->smellyExplored = std::vector<GraphNode>(); // reset the explored vector each time or something idk
    int depth = 0; // how deep we've searched so far
    
    // current and best costs for this search (not for A*)
    double cost = 0;
    double bestCost = 0;
    GraphNode bestPos;
    bestPos.x = (int)(MAX_X/2);
    bestPos.y = MAX_Y;
    double heading_to_gps = 0;

    // get the current time
    double timeNow = NOW;

    // if we don't have waypoints and it's time to use them
    if (this->waypoints.size() == 0  &&  timeNow > waypointTime  &&  waypointTime != 0) {
        RCLCPP_WARN(this->get_logger(), "TIME USE WAYPOINTS");

        // then assign waypoints and start moving
        this->waypoints = this->GetWaypoints();
        this->waypointTime = 0;
    // otherwise, if we don't have waypoints and it's not time to use them yet
    } else if (timeNow < this->waypointTime  &&  this->waypoints.size() == 0) {
        autonav_msgs::msg::PathingDebug pathingDebugMsg;
        pathingDebugMsg.waypoints = {};

        // let us know how long we have to wait before we start using waypoints 
        pathingDebugMsg.time_until_use_waypoints = this->waypointTime - timeNow;
        this->debugPublisher->publish(pathingDebugMsg);
    }

    // if we need to reset the color of the safety lights, do so
    if (this->resetWhen != -1  &&  timeNow > this->resetWhen  &&  this->mobility) {
        this->safetyPublisher->publish(this->GetSafetyLightsMsg(255, 255, 255));
        this->resetWhen = -1;
    }

    // if we do, however, have waypoints
    if (waypoints.size() > 0) {
        RCLCPP_WARN(this->get_logger(), "SMELLING THE WAYPOINTS");

        //FIXME this code is just straight copied pasted from the Python file
        auto next_waypoint = this->waypoints[0];
        auto north_to_gps = (next_waypoint.lat - this->gps_position.lat) * this->LATITUDE_LENGTH;
        auto west_to_gps = (this->gps_position.lon - next_waypoint.lon) * this->LONGITUDE_LENGTH; 

        heading_to_gps = fmod(atan2(west_to_gps, north_to_gps), (2 * PI));

        // if we've reached a waypoint
        if (GpsDistanceFormula(next_waypoint, this->gps_position) <= this->WAYPOINT_POP_DISTANCE) {
            this->waypoints.erase(this->waypoints.begin()); // remove it from the list

            // and let everyone know we've reached it using the safety lights
            this->safetyPublisher->publish(this->GetSafetyLightsMsg(0, 255, 0));
            this->resetWhen = timeNow + 1.5;
        }

        // debug information
        autonav_msgs::msg::PathingDebug debugMsg;
        debugMsg.desired_heading = heading_to_gps;
        debugMsg.desired_latitude = next_waypoint.lat;
        debugMsg.desired_longitude = next_waypoint.lon;
        debugMsg.distance_to_destination = GpsDistanceFormula(next_waypoint, this->gps_position);

        // convert waypoints ((x, y), (x, y)) to 1d array of (x, y, x, y) for ROS
        auto waypoint1Darr = std::vector<double>(waypoints.size() * 2);
        for (int i = 0, j = 0; i < (int)this->waypoints.size(); i++, j+=2) {
            waypoint1Darr[j] = this->waypoints[i].lat;
            waypoint1Darr[j+1] = this->waypoints[i].lon;
        }
        debugMsg.waypoints = waypoint1Darr;

        // and publish it
        this->debugPublisher->publish(debugMsg);
    }
    //=======================================================


    RCLCPP_WARN(this->get_logger(), "ACTUAL SMELLY BITS");
    RCLCPP_WARN(this->get_logger(), "FRONTIER SIZE: ");
    RCLCPP_WARN(this->get_logger(), std::to_string((int)this->frontier.size()).c_str());
    RCLCPP_WARN(this->get_logger(), "SMELLY FRONTIER SIZE: ");
    RCLCPP_WARN(this->get_logger(), std::to_string((int)smellyFrontier.size()).c_str());
    int iterations = 0;
    // smellification idk
    // while we haven't hit the max depth and still have nodes to explore
    while (depth < MAX_DEPTH && smellyFrontier.size() > 0) {
        // RCLCPP_WARN(this->get_logger(), "SMELLIFICATION DEPTH: ");
        // RCLCPP_WARN(this->get_logger(), std::to_string(depth).c_str());

        // we should explore those nodes
        for (int i = 0; i < (int)smellyFrontier.size(); i++) {
            auto node = smellyFrontier[i];

            // PRINT_NODE("SMELLING NODE: ", node);

            // cost is based on y coordinate and depth (favoring shorter paths)
            cost = (SMELLY_Y - node.y) * SMELLY_Y_COST  +  (depth * SMELLY_DEPTH_COST);

            if (this->waypoints.size() > 0) {
                //TODO what the heck do the 40 and 80 and what even is this doing hello figure this out please
                //TODO understand this line of code and leave a comment describing what it does
                double heading_error = abs(GetAngleDifference(heading_to_gps + atan2((MAX_X/2) - node.x, MAX_Y - node.y), heading_to_gps)) * 180 / PI;
                cost -= std::max(heading_error, (double)10);
            }

            // if the current cost is larger than our running best, then update that
            if (cost > bestCost) {
                bestCost = cost;
                bestPos = node;
            }

            // remove this node from the frontier because we've explored it
            smellyFrontier.erase(std::find(smellyFrontier.begin(), smellyFrontier.end(), node));
            
            // add it to the list of explored so we don't accidentally explore it again
            smellyExplored.push_back(node);

            // RCLCPP_WARN(this->get_logger(), "NODES:");

            //TODO fix this
            auto neighbors = this->GetNeighbors(node, false); // false so we don't check behind us

            // RCLCPP_WARN(this->get_logger(), "NUM NEIGHBORS:");
            // RCLCPP_WARN(this->get_logger(), std::to_string((int)neighbors.size()).c_str());

            // for the 3 nodes (to the left, to the right, and in front)
            for (GraphNode neighbor : neighbors) {
                // RCLCPP_WARN(this->get_logger(), neighbor.to_string().c_str());
                // check if we've explored it yet, and if we haven't
                // if neighbor not in explored: frontier.add(node)
                if (std::find(smellyFrontier.begin(), smellyFrontier.end(), neighbor) == smellyFrontier.end()) {
                    // add it to the frontier
                    smellyFrontier.push_back(neighbor);
                }
            }
            iterations++;
        }

        depth++;
    }

    RCLCPP_WARN(this->get_logger(), "NUM SMELLY ITERATIONS: ");
    RCLCPP_WARN(this->get_logger(), std::to_string(iterations).c_str());

    return bestPos;
}
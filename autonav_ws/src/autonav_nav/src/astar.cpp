#include "autonav_nav/astar.h"

#define PI 3.1415926535897932384626433

#define NOW std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()

// main initilization method (because we don't really have a constructor for reasons)
void AStarNode::init() {
    //TODO std::vector<std::vector<GraphNode>> map;
    //TODO make the file reading code so we can a list of waypoints and stuff

    // reserve space so we avoid constantly allocating and reallocating space
    frontier.reserve(100);
    closed.reserve(100);

    // === subscribers ===
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

    // === read waypoints from file ===
    waypointsFile.open(WAYPOINTS_FILENAME);

    // loop through the lines in the file
    std::string line;
    if (waypointsFile.is_open()) {
        while (getline(waypointsFile, line) ) {
            // label, latitude, longitude = line.split(",")
            // format is like label,lat,lon, right?
            // so label is [0:first comma]
            // latitude is [first comma:second to last comma] (because remember there's a trailing comma before the newline)
            // longitude is [second to last comma:end of string-1] (so we don't catch that last comma at the end)
            //TODO rewrite this as it is awful
            std::string label = line.substr(0, line.find(",")); //https://cplusplus.com/reference/string/string/find/
            double lat = std::stod(line.substr(line.find(","), line.substr(0, line.length()-1).rfind(","))); //https://cplusplus.com/reference/string/stod/
            double lon = std::stod(line.substr(line.substr(0, line.length()-1).rfind(","), line.length()-1));

            // if the vector doesn't exist yet in the dictionary, make it
            // if (!waypoints[label]) {
            //     waypoints[label] = std::vector<std::vector<double>>(5);
            // }

            // waypoints are stored like {"north":[(lat, lon), (lat, lon)]}
            // with lat, lon in sequential order
            waypointsDict[label].push_back({lat, lon});
        }
    }

    waypointsFile.close();
    // === /read waypoints ===

    // go ahead and initialize our waypointTime
    this->waypointTime = NOW + this->WAYPOINT_DELAY;
}

// system state callback function
void AStarNode::system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) {
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
    this->imu = autonav_msgs::msg::IMUData();
    this->position = geometry_msgs::msg::Pose();
    this->waypoints = std::vector<std::vector<double>>();
    this->waypointTime = NOW + this->WAYPOINT_DELAY;
}

// config update callback
void AStarNode::config_updated(json config) {
    //TODO
}

// return default config for initializing the display.html data
json AStarNode::get_default_config() {
    //TODO write
}


// main callback
void AStarNode::onConfigSpaceReceived(nav_msgs::msg::OccupancyGrid grid_msg) {
    grid = grid_msg;

    // perform A* every time we get new data, we don't need to otherwise
    this->DoAStar();
}

// gps data callback
void AStarNode::onPoseReceived(geometry_msgs::msg::Pose pos_msg) {
    this->position = pos_msg;
}

// imu data callback
void AStarNode::onImuReceived(autonav_msgs::msg::IMUData imu_msg) {
    //TODO figure out some kinda use for this?
    this->imu = imu_msg;
}

// main function of the A* node but not really
void AStarNode::DoAStar() {
    // find our goal node using smellification algorithm
    GraphNode goal = this->Smellification();

    // make the starting node (our initial position on the relative map)
    GraphNode start;
    start.x = MAX_X/2; // start in the middle
    start.y = MAX_Y; // start at the bottom, because arrays and indexing
    start.g_cost = 0;
    start.h_cost = DistanceFormula(start, goal);
    start.f_cost = start.g_cost + start.h_cost;

    // update the map with the grid data
    for (int x = 0; x < MAX_X; x++) {
        for (int y = 0; y < MAX_Y; y++) {
            // grid is a 1D array, map is a 2D array
            map[y][x] = grid.data[y+x];
        }
    }

    // perform A*
    auto pathMsg = AStarNode::ToPath(AStarNode::Search(start, goal));

    // only publish if we found a path
    if (pathMsg.poses.size() > 0) {
        // publish results
        this->pathPublisher->publish(pathMsg);

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

        // draw the cost map onto a debug image
        auto image = cv::Mat(MAX_X, MAX_Y, CV_8UC1, data); //TODO define data and double check size

        // loop through the image
        for (int x = 0; x < MAX_X; x++) {
            for (int y = 0; y < MAX_Y; y++) {
                // at the pixel at (x, y), draw the presence of an obstacle or not (*255 will make 1 into pure white)
                // image[x][y] = map[x][y] * 255;
                image.at<uchar>(y, x) = 128; // per https://docs.opencv.org/3.4/d5/d98/tutorial_mat_operations.html
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


// helper function to get the neighbors of the current node (excluding diagonals)
std::vector<GraphNode> AStarNode::GetNeighbors(GraphNode node) {
    std::vector<GraphNode> neighbors;

    std::vector<std::vector<double>> addresses = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}}; // addresses of the 4 edge-adjacent neighbors

    // loop through all the neighbors
    for(int i = 0; i < (int)addresses.size(); i++) {
        int neighbor_x = node.x + addresses[i][0];
        int neighbor_y = node.y + addresses[i][1];

        // check if it's in bounds
        if (0 < neighbor_x && neighbor_x < MAX_X) {
            if (0 < neighbor_y && neighbor_y < MAX_Y) {

                // check if it's traversable (ie not an obstacle, so != 1)
                if (map[neighbor_x][neighbor_y] == 0) {
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
    // pretty standard stuff
    node.g_cost = g_cost;
    node.h_cost = h_cost;
    node.f_cost = g_cost + h_cost;

    // parent is a pointer to a GraphNode that is the parent of this node
    node.parent = &current;
}

// convert a list of nodes to a ROS Path message
nav_msgs::msg::Path AStarNode::ToPath(std::vector<GraphNode> nodes) {
    nav_msgs::msg::Path pathMsg;
    pathMsg.header = std_msgs::msg::Header(); //TODO add actual header data or something

    //TODO initialize this with a good default size
    pathMsg.poses = std::vector<geometry_msgs::msg::PoseStamped>(); // wait this is a vector that's so sick

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

    return pathMsg;
}

// ======================================
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

// get the waypoints vector list thingy from the waypoints dictionary that stores all the waypoints we read from the file
std::vector<std::vector<double>> AStarNode::GetWaypoints() {
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
    } else {
        north = false;
    }


    if (this->system_mode == SCR::SystemMode::SIMULATION) {
        // return simulation waypoints if we're in simulation
        return this->waypointsDict["simulation1"];
    } else if (this->system_mode == SCR::SystemMode::COMPETITION) {
        // otherwise, if we're at competition, and we're facing north
        if (north) {
            // return the waypoints list for north
            return this->waypointsDict["compNorth"];
        } else {
            // otherwise south
            return this->waypointsDict["compSouth"];
        }
    } else {
        return this->waypointsDict["practice"];
    }
}

// get distance from GPS coordinates
double AStarNode::GpsDistanceFormula(std::vector<double> goal, std::vector<double> currPose) {
    // calculate lat and lon offsets
    auto north_to_gps = (goal[0] - currPose[0]) * this->LATITUDE_LENGTH;
    auto west_to_gps = (currPose[1] - goal[1]) * this->LONGITUDE_LENGTH;

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
    smellyFrontier = frontier; //FIXME I don't think copying over the frontier is right
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
        //FIXME this code is just straight copied pasted from the Python file
        auto next_waypoint = this->waypoints[0];
        auto north_to_gps = (next_waypoint[0] - this->gps_position[0]) * this->LATITUDE_LENGTH;
        auto west_to_gps = (this->gps_position[1] - next_waypoint[1]) * this->LONGITUDE_LENGTH; 

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
        debugMsg.desired_latitude = next_waypoint[0];
        debugMsg.desired_longitude = next_waypoint[1];
        debugMsg.distance_to_destination = GpsDistanceFormula(next_waypoint, this->gps_position);

        // convert waypoints ((x, y), (x, y)) to 1d array of (x, y, x, y) for ROS
        auto waypoint1Darr = std::vector<double>(waypoints.size() * 2);
        for (int i = 0, j = 0; i < (int)this->waypoints.size(); i++, j+=2) {
            waypoint1Darr[j] = this->waypoints[i][0];
            waypoint1Darr[j+1] = this->waypoints[i][1];
        }
        debugMsg.waypoints = waypoint1Darr;

        // and publish it
        this->debugPublisher->publish(debugMsg);
    }
    //=======================================================



    // smellification idk
    // while we haven't hit the max depth and still have nodes to explore
    while (depth < MAX_DEPTH && smellyFrontier.size() > 0) {
        // we should explore those nodes
        for (int i = 0; i < (int)smellyFrontier.size(); i++) {
            auto node = smellyFrontier[i];

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
            frontier.erase(std::find(frontier.begin(), frontier.end(), node));
            
            // add it to the list of explored so we don't accidentally explore it again
            // explored.push_back(node);

            //TODO fix this

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
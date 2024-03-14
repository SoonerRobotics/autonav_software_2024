// only include we need, to not accidentally double-include things
#include "autonav_nav/astar.h"

// main method (don't put in separate file else it no worky)
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    SCR::Node::run_node(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}

// initialization method (because we have no constructors) also reads GPS waypoints from a file
void AStarNode::init() {
    // field initialization (we actually have to do significantly less than I thought)
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


    // ROS stuff
    expandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20, std::bind(&AStarNode::onConfigSpaceReceived, this, std::placeholders::_1));
    poseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("/autonav/position", 20, std::bind(&AStarNode::onPoseReceived, this, std::placeholders::_1));
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/autonav/path", 20);


    // we've set everything up so now we're operating
    set_device_state(SCR::DeviceState::OPERATING);
}

// callback for GPS position
void AStarNode::onPositionReceived(geometry_msgs::msg::Pose msg) {

}

// callback for occupancy grid
void AStarNode::onOccupancyGridReceived(nav_msgs::msg::OccupancyGrid msg) {
    this->map = msg.data; // take the data and run with it

    // and do A*
    this->doAStar();
}

// main actual A* method
void AStarNode::doAStar() {
    auto goal = this->getGoalPoint();

    //TODO
}

// smellification algorithm to determine our goal node that we path plan to
GraphNode AStarNode::getGoalPoint() {
    GraphNode best;

    int depth = 0;

    while (depth < MAX_DEPTH) {
        //TODO
    }
}

// convert what's returned by A* into a publishable path and publish it
void AStarNode::publishPath() {
    //TODO
}

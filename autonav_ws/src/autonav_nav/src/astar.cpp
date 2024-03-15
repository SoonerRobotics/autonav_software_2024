// only include we need, to not accidentally double-include things
#include "autonav_nav/astar.h"

// https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/astar_rewrite/autonav_ws/src/autonav_nav/src/astar.cpp

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
    expandedSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20, std::bind(&AStarNode::onOccupancyGridReceived, this, std::placeholders::_1));
    poseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>("/autonav/position", 20, std::bind(&AStarNode::onPositionReceived, this, std::placeholders::_1));
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/autonav/path", 20);


    // we've set everything up so now we're operating
    set_device_state(SCR::DeviceState::OPERATING);
}

/*
 * on system state transition:
 *  - set getNewGpsCoords = true;
*/

// callback for GPS position
void AStarNode::onPositionReceived(geometry_msgs::msg::Pose msg) {

}

// callback for occupancy grid
void AStarNode::onOccupancyGridReceived(nav_msgs::msg::OccupancyGrid msg) {
    //FIXME msg.data is a vector, not an array
    this->map = msg.data; // take the data and run with it

    // and then publish our path
    this->publishPath();
}

// convert what's returned by A* into a publishable path and publish it
void AStarNode::publishPath() {
    // get the path to the goal
    std::vector<GraphNode> path = this->doAStar();

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


// main actual A* method
std::vector<GraphNode> AStarNode::doAStar() {
    // get our goal point
    GraphNode goal = this->getGoalPoint();
    
    // make our starting node
    GraphNode start;
    start.x = 0; //FIXME
    start.y = 0; //FIXME
    start.g_cost = 0;
    start.h_cost = 0;
    start.f_cost = 0;

    // initialize our frontier
    frontier.push(start);
    GraphNode current;
    
    // while there are still nodes we can explore, we should keep searching
    while (frontier.size() > 0) {
        current = frontier.top(); // get the node with the lowest cost
        frontier.pop(); // and remove it

        // if we're at the goal, then we should have a path reaching all the way back to the start
        if (current == goal) {
            break; // so break us out
        }

        // for each neighbor of the current node
        for (GraphNode neighbor : getNeighbors(current)) {
            // if we've already explored it,
            //TODO find a faster/better way to do this
            if (std::find(closed.begin(), closed.end(), neighbor) != closed.end()) { //https://stackoverflow.com/questions/571394/how-to-find-out-if-an-item-is-present-in-a-stdvector
                continue; // skip it
            }

            // otherwise, we should update the node
            neighbor.g_cost = current.g_cost + 1; // cost to move there is one more than where we are currently (because it is adjacent to us)
            neighbor.h_cost = this->heuristic(neighbor, goal); // additional cost
            neighbor.f_cost = neighbor.g_cost + neighbor.h_cost; // f_cost is total cost

            // if it's not in the frontier,
            if (neighbor not in frontier) { //FIXME
                // add it so we can explore its neighbors and expand the search
                frontier.push(neighbor);
            }
        }
    }

    // if we've broken out of the loop, then current is at the goal node
    //TODO return stuff
    std::vector<GraphNode> path;
    path.resize(100);
    GraphNode parent;
    while(true) {
        parent = current;
        current = *parent; //???
        path.push_back(parent);
    }

    // reverse the path because it's finish->start and it needs to be start->finish
    std::reverse(path.begin(), path.end()); // https://stackoverflow.com/questions/8877448/how-do-i-reverse-a-c-vector

    return path;
}

// smellification algorithm to determine our goal node that we path plan to
GraphNode AStarNode::getGoalPoint() {
    // if we're doing a new run, we might possibly be facing a different direction, and need to change which GPS coords we use to smellify and A*
    if (getNewGpsCoords) {
        //TODO update the GPS coords we're using
    } // otherwise we can continue to avoid doing that literally every main loop iteration

    // keep track of what position we're returning
    GraphNode best;
    best.f_cost = 99999;

    int depth = 0;
    while (depth < MAX_DEPTH && smellyFrontier.size() > 0) {
        // for each node on our frontier
        for (GraphNode node : smellyFrontier) {
            //TODO

            // for each neighbor of our current node
            for (GraphNode neighbor : getNeighbors(node)) {
                //TODO
            }

            // if the current node is cheaper to go to
            if (node.f_cost < best.f_cost) {
                // then replace the current best with the new best
                best.x = node.x;
                best.y = node.y;
                best.f_cost = node.f_cost;
            }
        }

        depth++;
    }

    return best;
}
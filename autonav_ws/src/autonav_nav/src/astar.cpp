#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"
#include "autonav_nav/astar.h"

//TODO
// std::vector<double> frontier;
// std::vector<double> closed;
// std::vector<std::vector<GraphNode>> map;
// also TODO make the file reading code so we can a list of waypoints and stuff
//TODO make smellification or whatever goal-finding heuristic algorithm thingy
//TODO use reserve() to not have to constantly resize all our vectors
SCR::Node::AstarNode() : SCR::Node("astar_fast") {
}

AStarNode::init() {
    map = nav_msgs::msg::MapMetaData();
    // TODO do this?
    // map.width = 100;
    // map.height = 100;
    // map.resolution = 0.1;
    // map.origin = geometry_msgs::msg::Pose();
    // map.origin.position.x = -10.0;
    // map.origin.position.y = -10.0;


    // left/right filtered subscribers
    leftExpandedSubscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded/left", this.onLeftReceived, 20);
    rightExpandedSubscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded/right", this.onRightReceived, 20);
    
    // localization data subscribers
    poseSubscriber = create_subscription<nav_msgs::msg::Position>("/autonav/position", this.onPoseReceived, 20);
    imuSubscriber = create_subscription<nav_msgs::msg::IMUData>("/autonav/imu", this.onImuReceived, 20);
    // raw_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>(directionify("/autonav/cfg_space/raw"), 20, std::bind(&ExpandifyNode::onConfigSpaceReceived, this, std::placeholders::_1));

    // === publishers ===
    pathPublisher = create_publisher<nav_msgs::msg::Path>("/autonav/path", 20);
    //TODO path publisher, safety lights publisher, debug publisher, debug image publisher

    //TODO callback timer

    // we've set everything up so now we're operating
    set_device_state(SCR::DeviceState::OPERATING);
}

    void config_updated(json newConfig) override
    {
        config = newConfig.template get<ExpandifyConfig>();
    }

    json get_default_config() override
    {
        ExpandifyConfig newConfig;
        newConfig.vertical_fov = 2.75;
        newConfig.horizontal_fov = 3;
        newConfig.map_res = 80.0f;
        return newConfig;
    }

    void system_state_transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
    {
        if (updated.state == SCR::SystemState::AUTONOMOUS && device_state == SCR::DeviceState::READY)
        {
            set_device_state(SCR::DeviceState::OPERATING);
        }

        if (updated.state != SCR::SystemState::AUTONOMOUS && device_state == SCR::DeviceState::OPERATING)
        {
            set_device_state(SCR::DeviceState::READY);
        }
    }

    void onConfigSpaceReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr cfg)
    {
        if (device_state != SCR::DeviceState::OPERATING || system_state != SCR::SystemState::AUTONOMOUS)
        {
            return;
        }

        std::vector<int8_t> cfg_space = std::vector<int8_t>(config.map_res * config.map_res);
        std::fill(cfg_space.begin(), cfg_space.end(), 0);

        for (int x = 0; x < config.map_res; x++)
        {
            for (int y = 1; y < config.map_res; y++)
            {
                if (cfg->data.at(x + y * config.map_res) > 0)
                {
                    for (Circle &circle : circles)
                    {
                        auto idx = (x + circle.x) + config.map_res * (y + circle.y);
                        auto expr_x = (x + circle.x) < config.map_res && (x + circle.x) >= 0;
                        auto expr_y = (y + circle.y) < config.map_res && (y + circle.y) >= 0;
                        if (expr_x && expr_y)
                        {
                            auto val = cfg_space.at(idx);
                            auto linear = 100 - ((circle.radius - noGoRange) / (maxRange - noGoRange) * 100);

                            if (circle.radius <= noGoRange)
                            {
                                cfg_space.at(idx) = 100;
                            }
                            else if (cfg_space.at(idx) <= 100 && val <= linear)
                            {
                                cfg_space.at(idx) = int(linear);
                            }
                        }
                    }
                }
            }
        }

        auto newSpace = nav_msgs::msg::OccupancyGrid();
        newSpace.info = map;
        newSpace.data = cfg_space;
        expanded_map_publisher->publish(newSpace);

        cv::Mat image = cv::Mat(config.map_res, config.map_res, CV_8UC1, cfg_space.data());
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        cv::resize(image, image, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);

        auto compressed = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
        compressed->header.stamp = this->now();
        compressed->header.frame_id = "map";
        compressed->format = "jpeg";
        debug_publisher->publish(*compressed);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_map_subscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr expanded_map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_publisher;

    nav_msgs::msg::MapMetaData map;
    std::string direction;

    float maxRange = 0.65;
    float noGoPercent = 0.70;
    int noGoRange = 0;
    std::vector<Circle> circles;
    ExpandifyConfig config;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto left_node = std::make_shared<ExpandifyNode>("left");
    auto right_node = std::make_shared<ExpandifyNode>("right");
    auto nodes = std::vector<std::shared_ptr<SCR::Node>>{left_node, right_node};
    SCR::Node::run_node(nodes);
    rclcpp::shutdown();
    return 0;
}

// hand references:
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
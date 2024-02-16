#include "astar.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarNode>();
    SCR::Node::run_node(node);
    rclcpp::shutdown();
    return 0;
}

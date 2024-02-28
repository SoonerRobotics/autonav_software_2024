#include "rclcpp/rclcpp.hpp"
#include "autonav_nav/astar.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}

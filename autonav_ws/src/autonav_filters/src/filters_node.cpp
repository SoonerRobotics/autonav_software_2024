#include "rclcpp/rclcpp.hpp"
#include "autonav_filters/filters.hpp"

int main (int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<FiltersNode>());
    // SCR::Node::run_node(std::make_shared<FiltersNode>());
    rclcpp::shutdown();
    return 0;
}
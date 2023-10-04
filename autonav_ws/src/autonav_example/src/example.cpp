#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "autonav_msgs/msg/test.hpp"
#include "autonav_core/core.h"
#include "rclcpp/rclcpp.hpp"

class ExampleNode : public SCR::AutonavNode
{
public:
    ExampleNode() : SCR::AutonavNode("example_cpp_node")
    {
        subscription_ = this->create_subscription<autonav_msgs::msg::Test>(
            "test_topic", 10, std::bind(&ExampleNode::topic_callback, this, std::placeholders::_1));
    }

    void topic_callback(const autonav_msgs::msg::Test::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

private:
    rclcpp::Subscription<autonav_msgs::msg::Test>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleNode>());
    rclcpp::shutdown();
    return 0;
}
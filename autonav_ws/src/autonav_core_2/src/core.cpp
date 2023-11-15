#include "autonav_core/core.h"
#include <unistd.h>

namespace SCR
{
    AutonavNode::AutonavNode(std::string node_name) : rclcpp::Node(node_name)
    {
    }

    AutonavNode::~AutonavNode()
    {
    }
}
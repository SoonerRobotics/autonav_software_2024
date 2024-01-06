#include "scr/node.h"
#include <unistd.h>

#define UNUSED(expr) do { (void)(expr); } while (0)

namespace SCR
{
	Node::Node(std::string node_name) : rclcpp::Node(node_name)
	{
		identifier = node_name;

		// Create subscriptions and clients
	}

	Node::~Node()
	{
	}
}
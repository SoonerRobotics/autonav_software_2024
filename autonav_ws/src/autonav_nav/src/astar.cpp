#pragma once

#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"
#include "astar.h"

//TODO
// std::vector<double> frontier;
// std::vector<double> closed;
//TODO map grid

//TODO search()

std::vector<GraphNode> AStarNode::get_neighbors(GraphNode node) {

}

double AStarNode::Heuristic(GraphNode node) {

}

//TODO reconstruct path

void AStarNode::update_node(GraphNode node, double g_cost, double h_cost, GraphNode current) {

}
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"

class AStarNode : public SCR::Node {
public:
    AStarNode() : Node("astar_fast"), count_(0) {
        //TODO
    }
private:
    //TODO
    std::vector<double> frontier;
    std::vector<double> closed;
    //TODO map grid

    //TODO search()

    std::vector<GraphNode> get_neighbors(GraphNode node);

    double heuristic(GraphNode node);

    //TODO reconstruct path

    void update_node(GraphNode node, double g_cost, double h_cost, GraphNode current);


    size_t count_;
}

struct GraphNode {
    int x;
    int y;

    double g_cost;
    double h_cost;
    double f_cost;

    GraphNode *parent;
}
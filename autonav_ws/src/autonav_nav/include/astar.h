#pragma once

#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"

class AStarNode : public SCR::Node {
public:
    AStarNode() : Node("astar_fast"), count_(0) {
        //TODO
    }
private:
    // open list
    std::vector<GraphNode> frontier;

    // have been searched list
    std::vector<GraphNode> closed;
    
    // full map grid thingamajig
    std::vector<std::vector<GraphNode>> map;

    // main actual search function
    std::vector<std::vector<GraphNode>> search(GraphNode start, GraphNode goal);

    // helper function to get the minecraft crafting table neighbors
    std::vector<GraphNode> get_neighbors(GraphNode node);

    // traverse the nodes backwards to the start and return that
    std::vector<GraphNode> reconstruct_path(GraphNode goal);

    // helper function to update a node's information
    void update_node(GraphNode node, double g_cost, double h_cost, GraphNode current);

    // our heuristic (h_cost), literally just distance formula
    double distance_formula(GraphNode current, GraphNode goal);


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
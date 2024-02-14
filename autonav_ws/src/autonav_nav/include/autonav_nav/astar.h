#pragma once

#include <math.h>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

struct GraphNode {
    int x;
    int y;

    double g_cost;
    double h_cost;
    double f_cost;

    GraphNode* parent;

    // define the less_than operator so we can use std::sort on the list
    // https://stackoverflow.com/questions/1380463/sorting-a-vector-of-custom-objects
    bool operator < (const GraphNode& other) const {
        return (f_cost < other.f_cost);
    }
};


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
    
    // full map grid thingamajig (0 is traversable/open, 1 is an obstacle)
    std::vector<std::vector<int>> map;

    // main actual search function
    std::vector<GraphNode> Search(GraphNode start, GraphNode goal);

    // helper function to get the minecraft crafting table neighbors
    std::vector<GraphNode> GetNeighbors(GraphNode node);

    // traverse the nodes backwards to the start and return that
    std::vector<GraphNode> ReconstructPath(GraphNode goal);

    // helper function to update a node's information
    void UpdateNode(GraphNode node, double g_cost, double h_cost, GraphNode current);

    // our heuristic (h_cost), literally just distance formula
    double DistanceFormula(GraphNode current, GraphNode goal);


    size_t count_;
    int MAX_X; //TODO assign these values (or figure out how to calculate them automagically)
    int MAX_Y;
    GraphNode start_node; //TODO initialize this one with wherever the heck we're starting at
};
#include "rclcpp/rclcpp.hpp"
#include "scr/node.hpp"
#include "autonav_nav/astar.h"

//TODO
// std::vector<double> frontier;
// std::vector<double> closed;
// std::vector<std::vector<GraphNode>> map;

//TODO search()


// helper function to get the minecraft crafting table neighbors
std::vector<GraphNode> AStarNode::GetNeighbors(GraphNode node) {
    std::vector<GraphNode> neighbors;

    // neighbors are just the surrounding nodes
    for (int x = -1; x < 1; x++) {
        for (int y = -1; y < 1; y++) {
            int neighbor_x = node.x + x;
            int neighbor_y = node.y + y;

            // check if it's in bounds
            if (0 < neighbor_x && neighbor_x < MAX_X) {
                if (0 < neighbor_y && neighbor_y < MAX_Y) {

                    // check if it's traversable (ie not an obstacle, so not == 1)
                    if (map[neighbor_x][neighbor_y] == 0) {
                        GraphNode node; //TODO this doesn't work I don't think
                        node.x = neighbor_x;
                        node.y = neighbor_y;

                        neighbors.push_back(node); // add it to the list
                    }
                }
            }
        }
    }

    return neighbors;
}


// standard distance formula (cartesian) between two nodse
double AStarNode::DistanceFormula(GraphNode current, GraphNode goal) {
    return sqrt(pow((current.x - goal.x), 2) + pow((current.y - goal.y), 2));
}


std::vector<GraphNode> AStarNode::ReconstructPath(GraphNode goal) {
    std::vector<GraphNode> path = {goal}; // the path starts at the goal node
    GraphNode current = goal; // and so we start at the goal node

    // while we haven't reached the start
    while (current.parent != &start_node) {
        // add the parent of the current node to the list
        path.push_back(*current.parent);

        // then move to the parent
        current = *current.parent;
    }

    // reverse the path (so that it's start to finish instead of finish to start)
    std::reverse(path.begin(), path.end()); //https://stackoverflow.com/questions/8877448/how-do-i-reverse-a-c-vector

    return path;
}


// update a node with new information
void AStarNode::UpdateNode(GraphNode node, double g_cost, double h_cost, GraphNode current) {
    node.g_cost = g_cost;
    node.h_cost = h_cost;
    node.f_cost = g_cost + h_cost;

    node.parent = &current;
}
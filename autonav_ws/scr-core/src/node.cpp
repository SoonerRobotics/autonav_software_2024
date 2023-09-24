#include "include/scr_core/node.h"

SCR::Node(std::string node_name) {
    //TODO
}

SCR::NodeState SCR::Node::GetNodeState() {
    return state_;
}

SCR::SystemState SCR::Node::GetSystemState() {
    return DISABLED; //TODO
}

void SCR::Node::SetSystemState(SCR::SystemState state) {
    //TODO
    return;
}

void SCR::Node::SetNodeState(SCR::NodeState state) {
    this.state_ = state;
}
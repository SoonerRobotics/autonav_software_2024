#include "include/scr_core/node.h"

SCR::Node(std::string node_name) {
    //TODO

    // copypastad from last year's code
    systemStateSubscriber = this->create_subscription<scr_msgs::msg::SystemState>("/scr/state/system", 100, std::bind(&Node::OnSystemState, this, std::placeholders::_1));
		

    this.state_ = DEAD;
    this.systemState_ = DISABLED;
}

SCR::Node::NodeState SCR::Node::GetNodeState() {
    return this.state_;
}

SCR::Node::SystemState SCR::Node::GetSystemState() {
    return this.systemState_;
}

void SCR::Node::SetSystemState(SCR::SystemState state) {
    //TODO
    return;
}

void SCR::Node::SetNodeState(SCR::NodeState state) {
    this.state_ = state;
}

void SCR::Node::OnSystemState(scr_msgs::msg::SystemState msg) {
    // This probably doesn't work but in case it does this is a nice simple little function so I'll leave it like this
    this.systemState_ = msg.state;
}
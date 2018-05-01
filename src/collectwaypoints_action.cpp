#include "supervisor/collectwaypoints_action.h"

CollectWaypointsAction::CollectWaypointsAction(ros::NodeHandle &nh, const std::string &name, const BT::TextParameters &params)
    : ActionNode(name), _nh(nh), _running(false) {
    assert(params.size() == 1);
    // there must be a single parameter
    _number = std::stoi(params.begin()->second);
}

void CollectWaypointsAction::collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (_route.size() < _number) {
        _route.push_back(msg->pose);
    }
}

BT::State CollectWaypointsAction::spin() {
    if (!_running) {
        _waypoints_sub  = _nh.subscribe("/waypoints", 1, &CollectWaypointsAction::collectWaypointsCallback, this);
        _running = true;
        ROS_INFO_STREAM("Subscription active");
        return BT::State::RUNNING;
    } else {
        if (_route.size() < _number) {
            ROS_INFO_STREAM("Waiting for waypoints...sleeping...");
            ros::Duration(1.0).sleep();
            return BT::State::RUNNING;
        } else {
            _waypoints_sub.shutdown();
            _blackboard->set("route", _route);
            _running = false;
            _route.clear();
            return BT::State::SUCCESS;
        }
    }
}

void CollectWaypointsAction::halt() {
    if (_running) {
        _waypoints_sub.shutdown();
        _route.clear();
        _blackboard->erase("route");
        _running = false;
    }
}

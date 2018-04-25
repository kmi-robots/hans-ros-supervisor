#include "supervisor/supervisor.h"
#include "behavior_tree/BehaviorTree.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "tf/tf.h"

Supervisor::Supervisor(BT::BlackBoard &blackboard): _blackboard(&blackboard), _route_initialized(false), _route_frame_id("/map") {
}

BT::State Supervisor::popWaypoint() {
    if (!_route_initialized) {
        if (!_blackboard->get("route", _route))
            return BT::State::FAILURE;
        _blackboard->erase("route");
        _route_initialized = true;
    }
    if (_route.empty()) {
        _route_initialized = false;
        return BT::State::FAILURE;
    }
    geometry_msgs::Pose point = std::move(_route.front());
    _route.pop_front();

    _blackboard->set("move_base/NextWaypoint/frame_id", _route_frame_id);
    _blackboard->set("move_base/NextWaypoint/pos_x", (double) point.position.x);
    _blackboard->set("move_base/NextWaypoint/pos_y", (double) point.position.y);
    _blackboard->set("move_base/NextWaypoint/theta", (double) tf::getYaw(point.orientation));

    ROS_INFO_STREAM("next waypoint is: (" << point.position.x << "," << point.position.y << ")");
    ROS_INFO_STREAM("there are " << _route.size() << " waypoints left");

    return BT::State::SUCCESS;
}

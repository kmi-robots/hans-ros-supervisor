#include "supervisor/supervisor.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "tf/tf.h"

Supervisor::Supervisor(std::shared_ptr<BT::Blackboard> blackboard)
            : _blackboard(blackboard), _route_initialized(false), _route_frame_id("/map") { }
            
BT::NodeStatus Supervisor::sleepOneSecond() {
    ROS_INFO_STREAM("Sleeping");
    ros::Duration(1).sleep();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Supervisor::popWaypoint() {
    if (!_route_initialized) {
        if (!_blackboard->get("route", _route))
            return BT::NodeStatus::FAILURE;
        _route_initialized = true;
    }
    if (_route.empty()) {
        _route_initialized = false;
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::Pose point = std::move(_route.front());
    _route.pop_front();

    _blackboard->set("move_base/NextWaypoint/frame_id", _route_frame_id);
    _blackboard->set("move_base/NextWaypoint/pos_x", (double) point.position.x);
    _blackboard->set("move_base/NextWaypoint/pos_y", (double) point.position.y);
    _blackboard->set("move_base/NextWaypoint/theta", (double) tf::getYaw(point.orientation));

    ROS_INFO_STREAM("next waypoint is: (" << point.position.x << "," << point.position.y << ")");
    ROS_INFO_STREAM("there are " << _route.size() << " waypoints left");

    return BT::NodeStatus::SUCCESS;
}

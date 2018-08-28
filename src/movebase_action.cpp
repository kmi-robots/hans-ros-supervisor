#include <tf/tf.h>
#include "supervisor/movebase_action.h"

MoveBaseAction::MoveBaseAction(const std::string &action_name, const BT::NodeParameters &params)
    : ActionNodeBase(action_name), _move_base("move_base", true) {
    assert(params.size() == 1);
    // there must be a single parameter
    _destination = params.at("destination");
}


BT::NodeStatus MoveBaseAction::tick() {
    if (!_move_base.isServerConnected()) {
        ROS_ERROR("MoveBaseAction failed because server is not connected");
        return BT::NodeStatus::FAILURE;
    }

    static int ID = 0;
    _goal.target_pose.header.stamp = ros::Time::now();
    _goal.target_pose.header.seq = ID++;

    double theta = 0.0;

    if (_destination == "BaseStation") {
        _goal.target_pose.header.frame_id = "/map";
        _goal.target_pose.pose.position.x = 2.0;
        _goal.target_pose.pose.position.y = 2.0;
        _goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    } else if (_destination == "NextWaypoint") {
        bool found = true;
        found &= blackboard()->get("move_base/NextWaypoint/frame_id",
                                    _goal.target_pose.header.frame_id);
        found &= blackboard()->get("move_base/NextWaypoint/pos_x",
                                    _goal.target_pose.pose.position.x);
        found &= blackboard()->get("move_base/NextWaypoint/pos_y",
                                    _goal.target_pose.pose.position.y);
        found &= blackboard()->get("move_base/NextWaypoint/theta", theta);

        _goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        if (!found) {
            ROS_ERROR("MoveBaseAction failed because part of the blackboard was empty");
            return BT::NodeStatus::FAILURE;
        }
    } else {
        ROS_ERROR("MoveBaseAction failed because _destination has unexpected value");
        return BT::NodeStatus::FAILURE;
    }

    ROS_INFO_STREAM("Send goal "<<_destination<<
                    ": x: "<<_goal.target_pose.pose.position.x<<
                    ", y: "<<_goal.target_pose.pose.position.y);
    _move_base.sendGoal(_goal);
    
    ROS_INFO_STREAM("Waiting for goal");
    _move_base.waitForResult();
    ROS_INFO_STREAM("Done");

    if (_move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

void MoveBaseAction::halt() {
    _move_base.cancelAllGoals();
}

#include <tf/tf.h>
#include "supervisor/movebase_action.h"

MoveBaseAction::MoveBaseAction(const std::string &action_name, const BT::TextParameters &params)
    : ActionNode(action_name), _move_base("move_base", true), _running(false) {
    assert(params.size() == 1);
    // there must be a single parameter
    _destination = params.begin()->second;
}


BT::State MoveBaseAction::spin() {
    if (!_move_base.isServerConnected()) {
        ROS_ERROR("MoveBaseAction failed because server is not connected");
        return BT::State::FAILURE;
    }

    ROS_INFO("Running value: %d", _running);

    if (!_running) {
        static int ID = 0;
        _goal.target_pose.header.stamp = ros::Time::now();
        _goal.target_pose.header.seq = ID++;

        double theta = 0.0;

        if (_destination == "BaseStation") {
            _goal.target_pose.header.frame_id = "/map";
            _goal.target_pose.pose.position.x = 0.0;
            _goal.target_pose.pose.position.y = 0.0;
            _goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        } else if (_destination == "NextWaypoint") {
            bool found = true;
            found &= _blackboard->get("move_base/NextWaypoint/frame_id",
                                      _goal.target_pose.header.frame_id);
            found &= _blackboard->get("move_base/NextWaypoint/pos_x",
                                      _goal.target_pose.pose.position.x);
            found &= _blackboard->get("move_base/NextWaypoint/pos_y",
                                      _goal.target_pose.pose.position.y);
            found &= _blackboard->get("move_base/NextWaypoint/theta", theta);

            _goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

            if (!found) {
                ROS_ERROR("MoveBaseAction failed because part of the blackboard was empty");
                return BT::State::FAILURE;
            }
        } else {
            ROS_ERROR("MoveBaseAction failed because _destination has unexpected value");
            return BT::State::FAILURE;
        }

        ROS_INFO("Send goal %s: %.2f %.2f %.2f", _destination.c_str(),
                 _goal.target_pose.pose.position.x,
                 _goal.target_pose.pose.position.y,
                 theta);
        _move_base.sendGoal(_goal);

        _running = true;
        return BT::State::RUNNING;
    } else { //running

        bool done = _move_base.waitForResult(ros::Duration(1.0));

        if (!done) {
            return BT::State::RUNNING;
        }
        _running = false;

        if (_move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            _blackboard->erase("move_base/NextWaypoint/frame_id");
            _blackboard->erase("move_base/NextWaypoint/pos_x");
            _blackboard->erase("move_base/NextWaypoint/pos_y");
            _blackboard->erase("move_base/NextWaypoint/theta");
            return BT::State::SUCCESS;
        } else {
            return BT::State::FAILURE;
        }
    }
}

void MoveBaseAction::halt() {
    if (_running) {
        _move_base.cancelAllGoals();
        _running = false;
    }
}

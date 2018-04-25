#include "tf/tf.h"
#include "supervisor/explore_action.h"

ExploreAction::ExploreAction(const std::string &action_name, const BT::TextParameters &params)
    : ActionNode(action_name), _explore("/Analyzer/explore", true), _running(false) {
    assert(params.size() == 1);
    // there must be a single parameter
    _goal.tag_number = std::stoi(params.begin()->second);
}


BT::State ExploreAction::spin() {
    if (!_explore.isServerConnected()) {
        ROS_ERROR("ExploreAction failed because server is not connected");
        return BT::State::FAILURE;
    }

    ROS_INFO("Running value: %d", _running);

    if (!_running) {
        ROS_INFO_STREAM("Starting exploration");
        _explore.sendGoal(_goal);
        _running = true;
        return BT::State::RUNNING;
    } else { //running

        bool done = _explore.waitForResult(ros::Duration(1.0));
        if (!done) {
            return BT::State::RUNNING;
        }
        _running = false;

        if (_explore.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return BT::State::SUCCESS;
        } else {
            return BT::State::FAILURE;
        }
    }
}

void ExploreAction::halt() {
    if (_running) {
        _explore.cancelAllGoals();
        _running = false;
    }
}

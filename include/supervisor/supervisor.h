#pragma once

#include <deque>
#include "behavior_tree_core/behavior_tree.h"
#include "Blackboard/blackboard_local.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

class Supervisor {
    public:
        Supervisor(std::shared_ptr<BT::Blackboard> blackboard);

        BT::NodeStatus popWaypoint();
        BT::NodeStatus sleepOneSecond();
    protected:
        std::shared_ptr<BT::Blackboard> _blackboard;
        ros::NodeHandle _nh;
        bool _route_initialized;
        std::deque<geometry_msgs::Pose> _route;
        std::string _route_frame_id;
};

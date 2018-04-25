#pragma once

#include <deque>
#include "behavior_tree/BehaviorTree.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

class Supervisor {
    public:
        Supervisor(BT::BlackBoard &blackboard);

        BT::State popWaypoint();
    protected:
        BT::BlackBoard *_blackboard;
        ros::NodeHandle _nh;
        bool _route_initialized;
        std::deque<geometry_msgs::Pose> _route;
        std::string _route_frame_id;
};

#pragma once

#include <deque>
#include <future>
#include "behavior_tree_core/behavior_tree.h"
#include "Blackboard/blackboard_local.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "supervisor/json.hpp"

#include <move_base_msgs/MoveBaseAction.h>
#include <room_analysis/ExploreAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

using json = nlohmann::json;

class FullSupervisor {
    public:
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        FullSupervisor(int number, std::string url);

        BT::NodeStatus popWaypoint();
        BT::NodeStatus sleepOneSecond();
        BT::NodeStatus collectWaypoints();
        BT::NodeStatus MoveBase();
        BT::NodeStatus Explore();
    private:
        void collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        std::future<std::string> invoke();
        void prepareRoute(json djin);
    protected:
        int _number;
        std::string _url;
        geometry_msgs::Pose _next_waypoint;
        ros::Subscriber _waypoints_sub;
        bool _route_initialized;
        std::deque<geometry_msgs::Pose> _route;
        std::string _route_frame_id;
        
        actionlib::SimpleActionClient<room_analysis::ExploreAction> _explore;
        room_analysis::ExploreGoal _goal;
        
        MoveBaseClient _move_base;
        move_base_msgs::MoveBaseGoal _mb_goal;
        tf::TransformListener _tf_listener;
};

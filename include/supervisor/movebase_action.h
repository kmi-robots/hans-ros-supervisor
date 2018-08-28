#pragma once
#include <behavior_tree_core/behavior_tree.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

using namespace BT;

class MoveBaseAction : public ActionNodeBase {
    public:
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

        MoveBaseAction(const std::string &action_name, const NodeParameters &params);
        
        static const NodeParameters& requiredNodeParameters() {
            static NodeParameters params = {{"destination","NextWaypoint"}};
            return params;
        }

        virtual NodeStatus tick() override;
        virtual void halt() override;
    private:
        MoveBaseClient  _move_base;
        std::string     _destination;
        move_base_msgs::MoveBaseGoal _goal;
        tf::TransformListener _tf_listener;
};

#pragma once
#include <behavior_tree/BehaviorTree.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

class MoveBaseAction : public BT::ActionNode<MoveBaseAction> {
    public:
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

        MoveBaseAction(const std::string &action_name, const BT::TextParameters &params);

        static const BT::TextParametersMap &RequiredParameters() {
            static BT::TextParametersMap required = { std::pair<std::string, BT::TreeNodeParameter>(
                    "destination", { BT::ParameterType::COMBO_TEXT, "NextWaypoint;BaseStation" })};
            return required;
        }

        virtual BT::State spin() override;
        virtual void halt() override;
    private:
        MoveBaseClient  _move_base;
        std::string     _destination;
        move_base_msgs::MoveBaseGoal _goal;
        bool _running;
        tf::TransformListener _tf_listener;
};

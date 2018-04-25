#pragma once
#include <behavior_tree/BehaviorTree.h>
#include <ros/ros.h>
#include <room_analysis/ExploreAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

class ExploreAction : public BT::ActionNode<ExploreAction> {
    public:
        ExploreAction(const std::string &action_name, const BT::TextParameters &params);

        static const BT::TextParametersMap &RequiredParameters() {
            static BT::TextParametersMap required = { std::pair<std::string, BT::TreeNodeParameter>(
                    "tag_id", { BT::ParameterType::INTEGER, "0" })};
            return required;
        }

        virtual BT::State spin() override;
        virtual void halt() override;

    private:
        actionlib::SimpleActionClient<room_analysis::ExploreAction> _explore;
        room_analysis::ExploreGoal _goal;
        bool _running;
};

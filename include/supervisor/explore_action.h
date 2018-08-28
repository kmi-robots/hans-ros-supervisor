#pragma once
#include <behavior_tree_core/behavior_tree.h>
#include <ros/ros.h>
#include <room_analysis/ExploreAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

using namespace BT;

class ExploreAction : public ActionNodeBase {
    public:
        ExploreAction(const std::string &action_name, const NodeParameters &params);

        static const NodeParameters& requiredNodeParameters() {
            static NodeParameters params = {{"classes","Heater"}};
            return params;
        }

        virtual NodeStatus tick() override;
        virtual void halt() override;

    private:
        actionlib::SimpleActionClient<room_analysis::ExploreAction> _explore;
        room_analysis::ExploreGoal _goal;
        bool _running;
};

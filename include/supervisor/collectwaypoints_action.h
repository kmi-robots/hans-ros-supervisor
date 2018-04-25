#pragma once
#include <deque>
#include "behavior_tree/BehaviorTree.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

class CollectWaypointsAction : public BT::ActionNode<CollectWaypointsAction> {
    public:
        CollectWaypointsAction(ros::NodeHandle &nh, const std::string &name, const BT::TextParameters &params);

        static const BT::TextParametersMap &RequiredParameters() {
            static BT::TextParametersMap required = { std::pair<std::string, BT::TreeNodeParameter>(
                    "number", { BT::ParameterType::INTEGER, "3" })};
            return required;
        }

        virtual BT::State spin() override;
        virtual void halt() override;
    private:
        ros::NodeHandle &_nh;
        ros::Subscriber _waypoints_sub;
        std::deque<geometry_msgs::Pose> _route;
        bool _running;
        int _number;

        void collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

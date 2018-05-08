#pragma once
#include <deque>
#include "behavior_tree/BehaviorTree.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "supervisor/json.hpp"

using json = nlohmann::json;

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
        const std::string ID = "collectWaypoints";

        void collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        std::future<std::string> invoke();
        void prepareRoute(json djin);
        geometry_msgs::Pose jsonToPose(json jpose);
};

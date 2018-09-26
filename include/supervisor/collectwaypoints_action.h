#pragma once
#include <deque>
#include <future>
#include "behavior_tree_core/behavior_tree.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "supervisor/json.hpp"

using json = nlohmann::json;

using namespace BT;

class CollectWaypointsAction : public ActionNodeBase {
    public:
        CollectWaypointsAction(const std::string &name, const NodeParameters &params);
        
        static const NodeParameters& requiredNodeParameters() {
            static NodeParameters params = {{"number","3"}};
            return params;
        }

        virtual NodeStatus tick() override;
        virtual void halt() override;
    private:
        ros::Subscriber _waypoints_sub;
        std::deque<geometry_msgs::Pose> _route;
        int _number;
        const std::string ID = "collectWaypoints";

        void collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        std::future<std::string> invoke();
        void prepareRoute(json djin);
//         geometry_msgs::Pose jsonToPose(json jpose);
};

#include "behavior_tree_core/bt_factory.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

using namespace BT;

namespace TestExample
{
    
void collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO_STREAM("Waypoint received");
}

BT::NodeStatus LongAction(const Blackboard::Ptr& blackboard)
{
    blackboard->set("waypoints_number", 20);
    ros::NodeHandle nh;
    ros::Subscriber _waypoints_sub  = nh.subscribe("/waypoints", 1, &collectWaypointsCallback);
    ROS_INFO_STREAM("Waiting for waypoints...");
    ros::Rate r(1);
    int counter = 0;
    while(counter < 20) {
        ros::spinOnce();
        r.sleep();
        counter++;
    }
    _waypoints_sub.shutdown();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ShortAction(const Blackboard::Ptr& blackboard)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000) );
    
    return NodeStatus::SUCCESS;
}

BT::NodeStatus CanFailAction(const Blackboard::Ptr& blackboard)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int counter = blackboard->get<int>("counter");
    counter--;
    blackboard->set("counter", counter);
    
    if(counter > 0)
        return NodeStatus::SUCCESS;
    else
        return NodeStatus::FAILURE;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerSimpleAction("LongAction", LongAction );
    factory.registerSimpleAction("ShortAction", ShortAction );
    factory.registerSimpleAction("CanFailAction", CanFailAction );
}

}

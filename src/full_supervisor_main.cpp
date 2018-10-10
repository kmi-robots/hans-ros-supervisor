#include "behavior_tree_core/xml_parsing.h"
#include "behavior_tree_core/bt_factory.h"
#include "supervisor/full_supervisor.h"

#include "supervisor/SendRule.h"
#include "ros/package.h"

#include "behavior_tree_logger/bt_cout_logger.h"

bool executeBehavior(supervisor::SendRule::Request  &req, supervisor::SendRule::Response &res) {
    ros::NodeHandle nh("~");
    std::string url;
    int number;
//     nh.getParam("file", filename);
    nh.param<int>("number", number, -1);
    nh.param<std::string>("url", url, "http://127.0.0.1:7070");

    BT::BehaviorTreeFactory bt_factory;
    auto blackboard = BT::Blackboard::create<BT::BlackboardLocal>();
    FullSupervisor supervisor(number, url);

    bt_factory.registerSimpleAction("Sleep", std::bind(&FullSupervisor::sleepOneSecond, &supervisor));
    bt_factory.registerSimpleAction("PopWaypoint", std::bind(&FullSupervisor::popWaypoint, &supervisor));
    bt_factory.registerSimpleAction("CollectWaypoints", std::bind(&FullSupervisor::collectWaypoints, &supervisor));
    bt_factory.registerSimpleAction("MoveBase", std::bind(&FullSupervisor::MoveBase, &supervisor));
    bt_factory.registerSimpleAction("Explore", std::bind(&FullSupervisor::Explore, &supervisor));
    
    std::string filename;
    filename = ros::package::getPath("supervisor");
    filename = filename + "/StateMachine.xml";
    ROS_INFO_STREAM("opening file " << filename);
    
    blackboard->set("waypoints_number", 3);
    
    auto tree = buildTreeFromFile(bt_factory, filename, blackboard);
    
    ROS_INFO_STREAM("execution started");
    
    BT::StdCoutLogger logger_cout(tree.root_node);
    
    supervisor._rule = req.rule;
    
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while( status == BT::NodeStatus::RUNNING ) {
        status = tree.root_node->executeTick();
    }
    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_action_supervisor");
    ros::NodeHandle nh("~");
    ros::ServiceServer service = nh.advertiseService("/execute_behavior", executeBehavior);
    ros::spin();
    
    return 0;
}

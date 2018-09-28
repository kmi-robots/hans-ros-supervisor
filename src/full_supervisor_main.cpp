#include "behavior_tree_core/xml_parsing.h"
#include "behavior_tree_core/bt_factory.h"
#include "supervisor/full_supervisor.h"
// #include "signal.h"
// #include "stdlib.h"

#include "behavior_tree_logger/bt_cout_logger.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_action_supervisor");

    ros::NodeHandle nh("~");
    std::string filename, url;
    int number;
    nh.getParam("file", filename);
    nh.param<int>("number", number, 3);
    nh.param<std::string>("url", url, "http://127.0.0.1:7070");

    BT::BehaviorTreeFactory bt_factory;
    auto blackboard = BT::Blackboard::create<BT::BlackboardLocal>();
    FullSupervisor supervisor(number, url);

    bt_factory.registerSimpleAction("Sleep", std::bind(&FullSupervisor::sleepOneSecond, &supervisor));
    bt_factory.registerSimpleAction("PopWaypoint", std::bind(&FullSupervisor::popWaypoint, &supervisor));
    bt_factory.registerSimpleAction("CollectWaypoints", std::bind(&FullSupervisor::collectWaypoints, &supervisor));
    bt_factory.registerSimpleAction("MoveBase", std::bind(&FullSupervisor::MoveBase, &supervisor));
    bt_factory.registerSimpleAction("Explore", std::bind(&FullSupervisor::Explore, &supervisor));
    
    if (filename.empty()) {
        char* pHome;
        pHome = getenv("HOME");
        filename = std::string(pHome);
        filename = filename + "/StateMachine.xml";
    }
    ROS_INFO_STREAM("opening file " << filename);
    //TODO temporary hardcode for testing
    std::string file = "/home/gianluca/catkin_ws/src/supervisor/StateMachine.xml";
    
    blackboard->set("waypoints_number", 3);
    
    auto res = buildTreeFromFile(bt_factory, file, blackboard);
    
    ROS_INFO_STREAM("execution started");
    
    BT::StdCoutLogger logger_cout(res.root_node);
    
    while (ros::ok()) {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
//     while( status == BT::NodeStatus::RUNNING ) {
        status = res.root_node->executeTick();
    }

    return 0;
}

#include "behavior_tree_core/xml_parsing.h"
#include "behavior_tree_core/bt_factory.h"
#include "supervisor/supervisor.h"
#include "supervisor/movebase_action.h"
#include "supervisor/collectwaypoints_action.h"
#include "supervisor/explore_action.h"
#include "signal.h"
#include "stdlib.h"

#include "behavior_tree_logger/bt_cout_logger.h"

static bool keepRunning = true;

int main(int argc, char **argv) {
    ros::init(argc, argv, "supervisor");

    ros::NodeHandle nh("~");
    std::string filename;
    nh.getParam("file", filename);

    BT::BehaviorTreeFactory bt_factory;
    auto blackboard = BT::Blackboard::create<BT::BlackboardLocal>();
    Supervisor supervisor(blackboard);

    bt_factory.registerSimpleAction("Sleep", std::bind(&Supervisor::sleepOneSecond, &supervisor));
    bt_factory.registerSimpleAction("PopWaypoint", std::bind(&Supervisor::popWaypoint, &supervisor));
    bt_factory.registerNodeType<MoveBaseAction>("MoveBase");
    bt_factory.registerNodeType<CollectWaypointsAction>("CollectWaypoints");
    bt_factory.registerNodeType<ExploreAction>("Explore");
    
    if (filename.empty()) {
        char* pHome;
        pHome = getenv("HOME");
        filename = std::string(pHome);
        filename = filename + "/StateMachine.xml";
    }
    ROS_INFO_STREAM("opening file " << filename);
    std::string file = "/home/gianluca/StateMachine.xml";
    auto res = buildTreeFromFile(bt_factory, file, blackboard);
    
    const TreeNode::Ptr& root_node = res.first;

    
    ROS_INFO_STREAM("execution started");
    
    BT::StdCoutLogger logger_cout(root_node.get());
    
    while (ros::ok()) {
        root_node->executeTick();
    }

    return 0;
}

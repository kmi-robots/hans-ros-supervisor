#include "supervisor/supervisor.h"
#include "supervisor/movebase_action.h"
#include "supervisor/collectwaypoints_action.h"
#include "supervisor/explore_action.h"
#include <signal.h>

using namespace BT;

static bool keepRunning = true;

BT::State SleepOneSecond() {
    ROS_INFO_STREAM("Sleeping");
    ros::Duration(1).sleep();
    return BT::State::SUCCESS;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "supervisor");

    ros::NodeHandle nh("~");
    std::string filename;
    nh.getParam("file", filename);

    BehaviorTreeFactory bt_factory;
    BT::BlackBoard blackboard;
    Supervisor supervisor(blackboard);

    bt_factory.registerSimpleAction("Sleep", SleepOneSecond);
    bt_factory.registerSimpleAction("PopWaypoint", &Supervisor::popWaypoint, supervisor);

    //------------------------------
    {
        ActionNodeCreator factory = [&](const std::string & name, const TextParameters & params) {
            return std::make_shared<MoveBaseAction>(name, params);
        };
        bt_factory.registerAction("MoveBase", MoveBaseAction::RequiredParameters(), factory);
    }

    {
        ActionNodeCreator factory = [&](const std::string & name, const TextParameters & params) {
            return std::make_shared<CollectWaypointsAction>(nh, name, params);
        };
        bt_factory.registerAction("CollectWaypoints", CollectWaypointsAction::RequiredParameters(), factory);
    }

    {
        ActionNodeCreator factory = [&](const std::string & name, const TextParameters & params) {
            return std::make_shared<ExploreAction>(name, params);
        };
        bt_factory.registerAction("Explore", ExploreAction::RequiredParameters(), factory);
    }
    //------------------------------

    // Load the actual tree from XML and create the instances of the Actions/Control Nodes.
    // It could be loaded from file.
    tinyxml2::XMLDocument bt_definition;
    if (filename.empty()) {
        filename = "/home/gianluca/catkin_ws/src/supervisor/StateMachine.xml";
    }
    ROS_INFO_STREAM("opening file " << filename);
    if (bt_definition.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        ROS_ERROR("Error parsing XML file %s", filename.c_str());
        return 1;
    }
    TreeNodePtr root_node = bt_factory.loadTreeFromXML(bt_definition);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    BT::AssignBlackboardRecursively(root_node, blackboard);

    while (keepRunning && ros::ok()) {
        std::cout << "------" << std::endl;
        BT::State res = root_node->executeSpin();
        std::cout << " result: " << res << std::endl;
        std::cout << "------" << std::endl;
    }
    return 0;
}

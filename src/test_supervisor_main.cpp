#include "test_example.h"
#include "behavior_tree_core/xml_parsing.h"
#include "behavior_tree_logger/bt_cout_logger.h"
#include "Blackboard/blackboard_local.h"

// clang-format off

const std::string xml_text = R"(

 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="MainSeq">
            <Action ID="LongAction" />
            <RetryUntilSuccesful num_attempts="10" >
                <Negation>
                    <Sequence name="ExplorationSeq">
                        <CanFailAction />
                        <ShortAction />
                    </Sequence>
                </Negation>
            </RetryUntilSuccesful>
            <ShortAction />
        </Sequence>
     </BehaviorTree>
 </root>
 )";

// clang-format on

using namespace BT;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_supervisor");
    BT::BehaviorTreeFactory factory;

    auto blackboard = Blackboard::create<BlackboardLocal>();
    blackboard->set("counter", 3);

    // register all the actions into the factory
    TestExample::RegisterNodes(factory);

    // Important: when the object tree goes out of scope, all the TreeNodes are destroyed
    auto tree = buildTreeFromText(factory, xml_text, blackboard);

    // Create some loggers
    StdCoutLogger   logger_cout(tree.root_node);
//     MinitraceLogger logger_minitrace(tree.root_node, "bt_trace.json");
//     FileLogger      logger_file(tree.root_node, "bt_trace.fbl");

    // Keep on ticking until you get either a SUCCESS or FAILURE state
    NodeStatus status = NodeStatus::RUNNING;
    while( status == NodeStatus::RUNNING )
    {
        status = tree.root_node->executeTick();
    }
    return 0;
}

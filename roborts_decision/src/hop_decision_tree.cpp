#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "behaviour_tree/behaviour_tree.h"
#include "executor/chassis_executor.h"
#include "blackboard/blackboard.h"

#include "hop_behaviour/back_boot_area_action.h"
#include "hop_behaviour/escape_action.h"
#include "hop_behaviour/chase_action.h"
#include "hop_behaviour/search_action.h"
#include "hop_behaviour/patrol_action.h"
#include "hop_behaviour/goal_action.h"
namespace roborts_decision
{

class HopTree
{
public:
    HopTree(const std::string &proto_file_path) : blackboard_ptr_(boost::make_shared<Blackboard>(full_path)), chassis_executor_(boost::make_shared<ChassisExecutor>())
    {
    }

protected:
    Blackboard::ConstPtr blackboard_ptr_;
    ChassisExecutor::ConstPtr chassis_executor_;
};
} // namespace roborts_decision

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_test_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    auto chassis_executor = new roborts_decision::ChassisExecutor();
    auto blackboard = new roborts_decision::Blackboard(full_path);

    roborts_decision::BackBootAreaAction back_boot_area_action(chassis_executor, blackboard);
    roborts_decision::ChaseAction chase_action(chassis_executor, blackboard);
    roborts_decision::SearchAction search_action(chassis_executor, blackboard);
    roborts_decision::EscapeAction escape_action(chassis_executor, blackboard);
    roborts_decision::PatrolAction patrol_action(chassis_executor, blackboard);
    roborts_decision::GoalAction goal_action(chassis_executor, blackboard);

    SelectorNode root_node("root_node", blackboard);
    root_node.SetLevel = 0;
    root_node.SetParent(NULL);
    PreconditionNode game_start("start", precondition_function = gameStart());
    SequenceNode sq_1("sq_1", blackboard);
    game_start.AddChildren(sq_1);
    sq_1.AddChildren({std::make_shared<BackBootAreaAction>(chassis_executor, blackboard)});

    return 0;
}
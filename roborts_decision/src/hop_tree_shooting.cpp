
#include "behavior_tree/behavior_tree_manager.h"

using namespace roborts_decision;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hop_decision_test_sooting_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    auto blackboard = std::make_shared<Blackboard>(full_path);

    auto tree_manager = std::make_shared<BehaviorTreeManager>(blackboard);

    tree_manager->AddCompositeNodes({"selector_main", "sequence_countdown", "selector_detect_and_shoot"});

    tree_manager->Connect("condition_countdown", "action_countdown");
    tree_manager->Connect("condition_game_start", "selector_detect_and_shoot");
    tree_manager->Connect("condition_enemy_detected", "action_shoot");
    tree_manager->Connect("selector_main", {"condition_game_start", "condition_countdown"});
    tree_manager->Connect("selector_detect_and_shoot", {"condition_enemy_detected"});
    tree_manager->Run("selector_main", 100);

    return 0;
}
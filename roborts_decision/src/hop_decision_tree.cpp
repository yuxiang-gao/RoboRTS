#include "behavior_tree/behavior_tree_manager.h"

using namespace roborts_decision;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "hop_decision_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto blackboard = std::make_shared<Blackboard>(full_path);

  auto tree_manager = std::make_shared<BehaviorTreeManager>(blackboard);
  tree_manager->AddCompositeNodes({"selector_main", "selector_patrol", "sequence_countdown", "parallel_engage"});

  tree_manager->Connect("condition_countdown", "sequence_countdown");
  tree_manager->Connect("condition_game_start", "selector_patrol");
  tree_manager->Connect("condition_reload", "action_reload");
  tree_manager->Connect("condition_bonus", "action_bonus");
  tree_manager->Connect("condition_enemy_detected", "parallel_engage");

  tree_manager->Connect("selector_patrol", {"condition_reload",
                                            "condition_enemy_detected",
                                            "condition_bonus",
                                            "action_patrol"});
  tree_manager->Connect("sequence_countdown", {"action_countdown"});
  //  "action_back_boot_area"});
  tree_manager->Connect("selector_main", {"condition_game_start", "condition_countdown"});

  tree_manager->Connect("parallel_engage", {"action_shoot",
                                            "action_chase"});

  tree_manager->Run("selector_main");

  return 0;
}
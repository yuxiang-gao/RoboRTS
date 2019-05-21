#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "behavior_tree/behavior_tree.h"
#include "executor/chassis_executor.h"
#include "blackboard/blackboard.h"

#include "hop_behavior/back_boot_area_action.h"
#include "hop_behavior/escape_action.h"
#include "hop_behavior/chase_action.h"
#include "hop_behavior/search_action.h"
#include "hop_behavior/patrol_action.h"
#include "hop_behavior/goal_action.h"
#include "hop_behavior/reload_action.h"
#include "hop_behavior/bonus_action.h"

#include "behavior_tree/behavior_tree_manager.h"

using namespace roborts_decision;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "hop_decision_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto blackboard = std::make_shared<Blackboard>(full_path);

  auto tree_manager = std::make_shared<BehaviorTreeManager>(blackboard);
  tree_manager->AddCompositeNodes({"sequence_reload", "selector_patrol"});
  tree_manager->Connect("condition_game_start", "selector_patrol");
  tree_manager->Connect("condition_reload", "action_reload");
  tree_manager->Connect("selector_patrol", {"condition_reload", "action_patrol"});
  tree_manager->Run("condition_game_start");

  // std::map<std::string, BehaviorNode::Ptr> conditions;
  // conditions["bonus"] = std::make_shared<PreconditionNode>("cond_bonus", blackboard, [blackboard]() { return blackboard->IsBonusAvailable(); }, AbortType::LOW_PRIORITY);

  // ActionNode::Ptr back_boot_area_action = std::make_shared<BackBootAreaAction>(chassis_executor, blackboard); //Done
  // ActionNode::Ptr chase_action = std::make_shared<ChaseAction>(chassis_executor, blackboard);
  // ActionNode::Ptr search_action = std::make_shared<SearchAction>(chassis_executor, blackboard);
  // ActionNode::Ptr escape_action = std::make_shared<EscapeAction>(chassis_executor, blackboard);
  // ActionNode::Ptr patrol_action = std::make_shared<PatrolAction>(chassis_executor, blackboard); //Done
  // ActionNode::Ptr goal_action = std::make_shared<GoalAction>(chassis_executor, blackboard);     //Done
  // ActionNode::Ptr reload_action = std::make_shared<ReloadAction>(chassis_executor, blackboard); //Done
  // ActionNode::Ptr bonus_action = std::make_shared<BonusAction>(chassis_executor, blackboard);   //TODO

  // DecoratorNode::Ptr cond_countdown = std::make_shared<PreconditionNode>("cond_countdown", blackboard, [blackboard]() { return blackboard->IsFiveSecondCD(); });
  // DecoratorNode::Ptr cond_game_start = std::make_shared<PreconditionNode>("cond_game_start", blackboard, [blackboard]() { return blackboard->IsGameStart(); });
  // // DecoratorNode::Ptr cond_bonus_unoccupied = std::make_shared<PreconditionNode>("cond_bonus_unoccupied", blackboard, [blackboard]() { return blackboard->IsBonusUnoccupied(); }, AbortType::LOW_PRIORITY);
  // // DecoratorNode::Ptr cond_bonus_occupied = std::make_shared<PreconditionNode>("cond_bonus_occupied", blackboard, [blackboard]() { return blackboard->IsBonusOccupied(); }, AbortType::LOW_PRIORITY);
  // DecoratorNode::Ptr cond_bonus = std::make_shared<PreconditionNode>("cond_bonus", blackboard, [blackboard]() { return blackboard->IsBonusAvailable(); }, AbortType::LOW_PRIORITY);
  // DecoratorNode::Ptr cond_enemy_detected = std::make_shared<PreconditionNode>("cond_enemy_detected", blackboard, [blackboard]() { return blackboard->IsEnemyDetected(); }, AbortType::LOW_PRIORITY);
  // DecoratorNode::Ptr cond_need_reload = std::make_shared<PreconditionNode>("cond_need_reload", blackboard, [blackboard]() { return blackboard->IsNeedReload(); }, AbortType::LOW_PRIORITY);

  // // CompositeNode::Ptr parallel_1 = std::make_shared<ParallelNode>("parallel_1", blackboard, 1);
  // CompositeNode::Ptr sequence_bonus = std::make_shared<SequenceNode>("sequence_bonus", blackboard);
  // CompositeNode::Ptr sequence_reload = std::make_shared<SequenceNode>("sequence_reload", blackboard);
  // CompositeNode::Ptr sequence_countdown = std::make_shared<SequenceNode>("sequence_reload", blackboard);

  // CompositeNode::Ptr selector_patrol = std::make_shared<SelectorNode>("selector_patrol", blackboard);
  // CompositeNode::Ptr selector_engage = std::make_shared<SelectorNode>("selector_engage", blackboard);

  // cond_game_start->SetChild(selector_patrol);
  // cond_need_reload->SetChild(sequence_reload);

  // selector_patrol->AddChildren({cond_need_reload,
  //                               patrol_action});
  // sequence_reload->AddChildren({reload_action, selector_patrol});

  // cond_countdown->SetChild(sequence_countdown);
  // cond_game_start->SetChild(selector_patrol);
  // cond_need_reload->SetChild(sequence_reload);
  // cond_bonus->SetChild(sequence_bonus);
  // cond_enemy_detected->SetChild(sequence_engage);

  // selector_patrol->AddChildren({cond_bonus,
  //                               cond_need_reload,
  //                               cond_enemy_detected,
  //                               patrol_action});
  // sequence_bonus->AddChildren({bonus_action, selector_patrol});
  // sequence_reload->AddChildren({reload_action, selector_patrol});

  // if (blackboard->IsFiveSecondCD())
  // {
  //   blackboard->TurnOnFricWheel();
  // }

  // roborts_decision::BehaviorTree behavior_tree(cond_game_start, 100);
  // behavior_tree.Run();
  return 0;
}
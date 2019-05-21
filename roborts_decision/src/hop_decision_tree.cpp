#include "behavior_tree/behavior_tree_manager.h"
using namespace roborts_decision;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hop_decision_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto blackboard = std::make_shared<Blackboard>(full_path);

  auto back_boot_area_action = std::make_shared<BackBootAreaAction>(blackboard); //Done
  auto chase_action = std::make_shared<ChaseAction>(blackboard);
  auto search_action = std::make_shared<SearchAction>(blackboard);
  auto escape_action = std::make_shared<EscapeAction>(blackboard);
  auto patrol_action = std::make_shared<PatrolAction>(blackboard); //Done
  auto goal_action = std::make_shared<GoalAction>(blackboard);     //Done
  auto reload_action = std::make_shared<ReloadAction>(blackboard); //Done
  auto bonus_action = std::make_shared<BonusAction>(blackboard);   //TODO

  auto cond_countdown = std::make_shared<PreconditionNode>("cond_countdown", blackboard, [blackboard]() { return blackboard->IsFiveSecondCD(); });
  auto cond_game_start = std::make_shared<PreconditionNode>("cond_game_start", blackboard, [blackboard]() { return blackboard->IsGameStart(); });
  // auto cond_bonus_unoccupied = std::make_shared<PreconditionNode>("cond_bonus_unoccupied", blackboard, [blackboard]() { return blackboard->IsBonusUnoccupied(); }, AbortType::LOW_PRIORITY);
  // auto cond_bonus_occupied = std::make_shared<PreconditionNode>("cond_bonus_occupied", blackboard, [blackboard]() { return blackboard->IsBonusOccupied(); }, AbortType::LOW_PRIORITY);
  auto cond_bonus = std::make_shared<PreconditionNode>("cond_bonus", blackboard, [blackboard]() { return blackboard->IsBonusAvailable(); }, AbortType::LOW_PRIORITY);
  auto cond_enemy_detected = std::make_shared<PreconditionNode>("cond_enemy_detected", blackboard, [blackboard]() { return blackboard->IsEnemyDetected(); }, AbortType::LOW_PRIORITY);
  auto cond_need_reload = std::make_shared<PreconditionNode>("cond_need_reload", blackboard, [blackboard]() { return blackboard->IsNeedReload(); }, AbortType::LOW_PRIORITY);

  // auto parallel_1 = std::make_shared<ParallelNode>("parallel_1", blackboard, 1);
  auto sequence_bonus = std::make_shared<SequenceNode>("sequence_bonus", blackboard);
  auto sequence_reload = std::make_shared<SequenceNode>("sequence_reload", blackboard);
  auto sequence_countdown = std::make_shared<SequenceNode>("sequence_reload", blackboard);

  auto selector_patrol = std::make_shared<SelectorNode>("selector_patrol", blackboard);
  auto selector_engage = std::make_shared<SelectorNode>("selector_engage", blackboard);

  cond_game_start->SetChild(selector_patrol);
  cond_need_reload->SetChild(reload_action);

  selector_patrol->AddChildren({cond_need_reload,
                                patrol_action});
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

  roborts_decision::BehaviorTree behavior_tree(cond_game_start, 100);
  behavior_tree.Run();
  return 0;
}
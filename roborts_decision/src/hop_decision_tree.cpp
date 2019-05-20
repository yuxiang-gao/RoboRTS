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

using namespace roborts_decision;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  BehaviorNode::Ptr chassis_executor = std::make_shared<ChassisExecutor>();
  BehaviorNode::Ptr blackboard = std::make_shared<Blackboard>(full_path);

  BehaviorNode::Ptr back_boot_area_action = std::make_shared<BackBootAreaAction>(chassis_executor, blackboard); //Done
  BehaviorNode::Ptr chase_action = std::make_shared<ChaseAction>(chassis_executor, blackboard);
  BehaviorNode::Ptr search_action = std::make_shared<SearchAction>(chassis_executor, blackboard);
  BehaviorNode::Ptr escape_action = std::make_shared<EscapeAction>(chassis_executor, blackboard);
  BehaviorNode::Ptr patrol_action = std::make_shared<PatrolAction>(chassis_executor, blackboard); //Done
  BehaviorNode::Ptr goal_action = std::make_shared<GoalAction>(chassis_executor, blackboard);     //Done
  BehaviorNode::Ptr reload_action = std::make_shared<ReloadAction>(chassis_executor, blackboard); //Done
  BehaviorNode::Ptr bonus_action = std::make_shared<BonusAction>(chassis_executor, blackboard);   //TODO

  BehaviorNode::Ptr cond_countdown = std::make_shared<PreconditionNode>("cond_countdown", blackboard->IsFiveSecondCD, blackboard, AbortType::SELF);
  BehaviorNode::Ptr cond_game_start = std::make_shared<PreconditionNode>("cond_game_start", blackboard->IsGameStart, blackboard, AbortType::SELF);
  BehaviorNode::Ptr cond_bonus_unoccupied = std::make_shared<PreconditionNode>("cond_bonus_unoccupied", blackboard->IsBonusUnoccupied, blackboard, AbortType::SELF);
  BehaviorNode::Ptr cond_bonus_occupied = std::make_shared<PreconditionNode>("cond_bonus_occupied", blackboard->IsBonusOccupied, blackboard, AbortType::SELF);
  BehaviorNode::Ptr cond_enemy_detected = std::make_shared<PreconditionNode>("cond_enemy_detected", blackboard->IsEnemyDetected, blackboard, AbortType::SELF);
  BehaviorNode::Ptr cond_need_reload = std::make_shared<PreconditionNode>("cond_need_reload", blackboard->IsNeedReload, blackboard, AbortType::SELF);

  BehaviorNode::Ptr parallel_1 = std::make_shared<ParallelNode>("parallel_1", blackboard, 1);
  BehaviorNode::Ptr sequence_1 = std::make_shared<SequenceNode>("sequence_1", blackboard);
  BehaviorNode::Ptr selector_1 = std::make_shared<SelectorNode>("selector_1", blackboard);
  BehaviorNode::Ptr selector_2 = std::make_shared<SelectorNode>("selector_2", blackboard);

  cond_game_start->SetChild(selector_1);
  cond_need_reload->SetChild(sequence_2);

  selector_1->AddChildren({
      cond_
          patrol_action,

  });

  // if (blackboard->IsFiveSecondCD())
  // {
  //   blackboard->TurnOnFricWheel();
  // }

  // roborts_decision::BehaviorTree behavior_tree(pointer_Ptr(SE1), 100);
  return 0;
}
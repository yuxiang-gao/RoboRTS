#ifndef ROBORTS_DECISION_RELOAD_ACTION_H
#define ROBORTS_DECISION_RELOAD_ACTION_H

#include "io/io.h"
#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "behaviour_tree/behaviour_tree.h"
#include "blackboard/blackboard.h"
#include "utils/line_iterator.h"

namespace roborts_decision
{
class GoalAction : public ActionNode
{
public:
  GoalAction(ChassisExecutor::Ptr &chassis_executor,
             Blackboard::Ptr &blackboard) : ActionNode("goal_action", blackboard),
                                            chassis_executor_(chassis_executor) {}

  void OnInitialize()
  {
  }

  void OnTerminate(BehaviorState state)
  {
    chassis_executor_->Cancel();
  }

  BehaviorState Update()
  {
    auto executor_state = chassis_executor_->Update();
    if (executor_state != BehaviorState::RUNNING)
    {
      if (blackboard_->IsNewGoal())
      {
        chassis_executor_->Execute(blackboard_->GetGoal());
        return BehaviourState::SUCCESS;
      }
    }
    return BehaviorState::RUNNING;
  }

  ~GoalAction() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_RELOAD_ACTION_H

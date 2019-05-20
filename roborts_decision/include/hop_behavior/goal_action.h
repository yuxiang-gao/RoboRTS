#ifndef ROBORTS_DECISION_GOAL_ACTION_H
#define ROBORTS_DECISION_GOAL_ACTION_H

#include "io/io.h"
#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"
#include "utils/line_iterator.h"

namespace roborts_decision
{
class GoalAction : public ActionNode
{
public:
  GoalAction(const ChassisExecutor::Ptr &chassis_executor,
             const Blackboard::Ptr &blackboard) : ActionNode("goal_action", blackboard),
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
      if (blackboard_ptr_->IsNewGoal())
      {
        chassis_executor_->Execute(blackboard_ptr_->GetGoal());
        return BehaviorState::RUNNING;
      }
    }
    return executor_state;
  }

  ~GoalAction() = default;

private:
  //! executor
  const ChassisExecutor::Ptr chassis_executor_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_GOAL_ACTION_H

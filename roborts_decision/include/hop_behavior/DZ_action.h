#ifndef ROBORTS_DECISION_DZ_ACTION_H
#define ROBORTS_DECISION_DZ_ACTION_H

#include "io/io.h"
#include <ros/ros.h>
#include <chrono>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"
#include "utils/line_iterator.h"

namespace roborts_decision
{
class DZAction : public ActionNode
{
public:
  GoalAction(ChassisExecutor::Ptr &chassis_executor,
             Blackboard::Ptr &blackboard) : ActionNode("DZ_action", blackboard),
                                            chassis_executor_(chassis_executor) {}

    void OnInitialize()
  {
        DZ_goal_.header.frame_id = "map";
        DZ_goal_.pose.orientation.x = 0;
        DZ_goal_.pose.orientation.y = 0;
        DZ_goal_.pose.orientation.z = 0;
        DZ_goal_.pose.orientation.w = 1;
        DZ_goal_.pose.position.x = 0;
        DZ_goal_.pose.position.y = 0;
        DZ_goal_.pose.position.z = 0;
        DZ_goal_ = true;

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
      /*if (blackboard_->IsNewGoal())
      {
        chassis_executor_->Execute(blackboard_->GetGoal());
        return Behaviortate::SUCCESS;
      }*/
        chassis_executor_->Execute(blackboard_->GetDZGoal());
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end_time-start_time;
        while (diff <= 7)
        {
            std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = end-start;
        }
        return Behaviortate::SUCCESS;
    }
    return BehaviorState::RUNNING;
  }

  ~GoalAction() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! DZ goal
  geometry_msgs::PoseStamped DZ_goal_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_DZ_ACTION_H

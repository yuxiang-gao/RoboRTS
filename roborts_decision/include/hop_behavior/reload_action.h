#ifndef ROBORTS_DECISION_RELOAD_ACTION_H
#define ROBORTS_DECISION_RELOAD_ACTION_H
#include <chrono>
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
  ReloadAction(std::string robot_name, const ChassisExecutor::Ptr &chassis_executor,
             const Blackboard::Ptr &blackboard) : ActionNode("reload_action", blackboard),
                                            chassis_executor_(chassis_executor) {}

    void OnInitialize()
  {
        reload_goal_.header.frame_id = "map";
        reload_goal_.pose.orientation.x = 0;
        reload_goal_.pose.orientation.y = 0;
        reload_goal_.pose.orientation.z = 0;
        reload_goal_.pose.orientation.w = 1;
        reload_goal_.pose.position.x = 0;
        reload_goal_.pose.position.y = 0;
        reload_goal_.pose.position.z = 0;
        reload_goal_ = true;

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
        chassis_executor_->Execute(blackboard_ptr_->GetReloadGoal());
        blackboard->RefreePublishSupply(robot_name_)
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

  std::string robot_name_;

  //! reload goal
  geometry_msgs::PoseStamped reload_goal_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_RELOAD_ACTION_H

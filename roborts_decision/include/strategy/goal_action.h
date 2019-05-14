#ifndef ROBORTS_DECISION_GOAL_ACTION_H
#define ROBORTS_DECISION_GOAL_ACTION_H


#include "io/io.h"
#include <ros/ros.h>

#include "executor/chassis_executor.h"


#include "behaviour_tree/behaviour_tree.h"
#include "blackboard/blackboard.h"

#include "example_behavior/line_iterator.h"


namespace roborts_decision {
class GoalAction: public ActionNode {
 public:
  GoalAction(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard) : ActionNode("goal_act", blackboard),
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) { }

  void OnInitialize() {
    if(blackboard_->IsNewGoal()){
      chassis_executor_->Execute(blackboard_->GetGoal());
    }
  }
  
  BehaviorState Run()
    {
        auto executor_state = Update();
        behavior_state_ = ActionNode::Run();
        return behavior_state_;
    }

  void OnTerminate(BehaviorState state) {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~GoalBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;

};
}

#endif //ROBORTS_DECISION_GOAL_ACTION_H

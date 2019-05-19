#ifndef ROBORTS_DECISION_PATROL_ACTION_H
#define ROBORTS_DECISION_PATROL_ACTION_H

#include "io/io.h"

#include "blackboard/blackboard.h"
#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_state.h"

#include "utils/line_iterator.h"

namespace roborts_decision
{
class PatrolAction : public ActionNode
{
public:
  PatrolAction(const ChassisExecutor::Ptr &chassis_executor,
               const Blackboard::Ptr &blackboard) : ActionNode("patrol_act", blackboard), chassis_executor_(chassis_executor)
  {
  }

  void OnInitialize()
  {
    patrol_count_ = 0;
    point_size_ = 0;

    point_size_ = (unsigned int)(blackboard_ptr_->decision_config.point().size());
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++)
    {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = blackboard_ptr_->decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = blackboard_ptr_->decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = blackboard_ptr_->decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(blackboard_ptr_->decision_config.point(i).roll(),
                                                              blackboard_ptr_->decision_config.point(i).pitch(),
                                                              blackboard_ptr_->decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }
  }

  BehaviorState Update()
  {
    auto executor_state = chassis_executor_->Update();
    if (executor_state != BehaviorState::RUNNING)
    {
      if (patrol_goals_.empty())
      {
        ROS_ERROR("patrol goal is empty");
        return BehaviorState::FAILURE;
      }

      ROS_INFO("send goal");
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
      return BehaviorState::SUCCESS;
    }
    return BehaviorState::RUNNING;
  }

  void OnTerminate(BehaviorState state)
  {
    chassis_executor_->Cancel();
  }



  ~PatrolAction() = default;

private:
  //! executor
  const ChassisExecutor::Ptr chassis_executor_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_PATROL_ACTION_H

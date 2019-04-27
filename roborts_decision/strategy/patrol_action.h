#ifndef ROBORTS_DECISION_PATROL_ACTION_H
#define ROBORTS_DECISION_PATROL_ACTION_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class PatrolAction : public ActionNode {
 public:
  PatrolAction(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,) : ActionNode("patrol_act", blackboard), chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    patrol_count_ = 0;
    point_size_ = 0;

    point_size_ = (unsigned int)(blackboard_ptr_->decision_config.point().size());
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = blackboard_ptr_->decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = blackboard_ptr_->decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = blackboard_ptr_->decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(blackboard_ptr_->decision_config.point(i).roll(),
                                                              blackboard_ptr_->decision_config.point(i).pitch(),
                                                              blackboard_ptr_->decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = blackboard_ptr_->quaternion.x();
      patrol_goals_[i].pose.orientation.y = blackboard_ptr_->quaternion.y();
      patrol_goals_[i].pose.orientation.z = blackboard_ptr_->quaternion.z();
      patrol_goals_[i].pose.orientation.w = blackboard_ptr_->quaternion.w();

  }

  void OnIntialize() 
  {
	  if (patrol_goals_.empty()) {
		ROS_ERROR("patrol goal is empty");
		return;
	  }

	  std::cout << "send goal" << std::endl;
	  chassis_executor_->Execute(patrol_goals_[patrol_count_]);
	  patrol_count_ = ++patrol_count_ % point_size_;

    }
  BehaviorState Run()
    {
        auto executor_state = Update();
		std::cout << "state: " << (int)(executor_state) << std::endl;
        behavior_state_ = ActionNode::Run();
        return behavior_state_;
    }
  void OnTerminate(BehaviorState state)
  {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }


  ~PatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

};
}

#endif //ROBORTS_DECISION_PATROL_ACTION_H

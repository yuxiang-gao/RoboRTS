#ifndef ROBORTS_DECISION_RELOAD_ACTION_H
#define ROBORTS_DECISION_RELOAD_ACTION_H
#include <chrono>
#include "io/io.h"
#include <ros/ros.h>

#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"
#include "utils/line_iterator.h"

namespace roborts_decision
{
class ReloadAction : public ActionNode
{
public:
  ReloadAction(const Blackboard::Ptr &blackboard) : ActionNode("action_reload", blackboard),
                                                    chassis_executor_(blackboard->chassis_executor) {}

  void OnInitialize()
  {
    reload_position_.header.frame_id = "/map";
    reload_position_.pose.orientation.x = 0;
    reload_position_.pose.orientation.y = 0;
    reload_position_.pose.orientation.z = -0.7071;
    reload_position_.pose.orientation.w = 0.7071;
    reload_position_.pose.position.x = 4;
    reload_position_.pose.position.y = 4.5;
    reload_position_.pose.position.z = 0;
    chassis_cmd_sent_ = false;
    supply_cmd_sent_ = false;
    supply_start_time_ = 0;
  }

  void OnTerminate(BehaviorState state)
  {
    chassis_executor_->Cancel();
    chassis_cmd_sent_ = false;
    supply_cmd_sent_ = false;
    supply_start_time_ = 0;
    LogState(state);
  }

  BehaviorState Update()
  {
    auto executor_state = chassis_executor_->Update();
    // ROS_INFO_STREAM_THROTTLE(1, "Executor State: " << (int)executor_state);
    // ROS_INFO_STREAM("cond " << (int)executor_state << " " << (int)chassis_cmd_sent_);
    if (executor_state != BehaviorState::RUNNING && !chassis_cmd_sent_)
    {
      chassis_cmd_sent_ = true;
      chassis_executor_->Execute(reload_position_);
    }
    else if (chassis_cmd_sent_ && executor_state == BehaviorState::FAILURE)
    {
      return BehaviorState::FAILURE;
    }
    else if (chassis_cmd_sent_ && executor_state == BehaviorState::SUCCESS)
    {
      int supplier_status = blackboard_ptr_->GetSupplierStatus();
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      auto linear_distance = blackboard_ptr_->GetDistance(robot_map_pose, reload_position_);

      // ROS_INFO_STREAM_THROTTLE(1, "distance: " << linear_distance);
      // ROS_INFO_STREAM_THROTTLE(1, "Supply Status: " << supplier_status);

      if (!supply_cmd_sent_)
      {
        if (linear_distance <= 0.1)
        {
          ROS_INFO_STREAM("send supply signal");
          blackboard_ptr_->Supply();
          supply_cmd_sent_ = true;
        }
      }
      else if (supply_cmd_sent_ && supply_start_time_ == 0)
      {
        if (supplier_status == 2)
        {
          supply_start_time_ = ros::Time::now().toSec();
        }
        ROS_INFO("START TIME %f", ros::Time::now().toSec());
      }
      else if (supply_cmd_sent_ && supply_start_time_ != 0)
      {
        ROS_INFO("LOADING TIME %f", ros::Time::now().toSec() - supply_start_time_);
        if (supplier_status == 0)
        {
          return BehaviorState::SUCCESS;
        }
        else if (ros::Time::now().toSec() - supply_start_time_ > 30)
        {
          ROS_WARN("Reloading time out! LEAVE");
          return BehaviorState::FAILURE;
        }
      }
    }
    return BehaviorState::RUNNING;
  }

  ~ReloadAction() = default;

private:
  //! executor
  const ChassisExecutor::Ptr chassis_executor_;

  std::string robot_name_;

  //! reload goal
  geometry_msgs::PoseStamped reload_position_;

  bool chassis_cmd_sent_;
  bool supply_cmd_sent_;
  double supply_start_time_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_RELOAD_ACTION_H

#ifndef ROBORTS_DECISION_BONUS_ACTION_H
#define ROBORTS_DECISION_BONUS_ACTION_H
#include <chrono>
#include "io/io.h"
#include <ros/ros.h>

#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"
#include "utils/line_iterator.h"

namespace roborts_decision
{
class BonusAction : public ActionNode
{
public:
  BonusAction(const Blackboard::Ptr &blackboard) : ActionNode("action_bonus", blackboard),
                                                   chassis_executor_(blackboard->chassis_executor) {}

  void OnInitialize()
  {
    bonus_position_.header.frame_id = "/map";
    bonus_position_.pose.orientation.x = 0;
    bonus_position_.pose.orientation.y = 0;
    bonus_position_.pose.orientation.z = 0.7071068; // yaw = 90
    bonus_position_.pose.orientation.w = 0.7071068; // yaw = 90
    bonus_position_.pose.position.x = 6.3;
    bonus_position_.pose.position.y = 1.75;
    bonus_position_.pose.position.z = 0;
    bonus_right_ = bonus_position_; // yaw = 45 degree
    bonus_left_ = bonus_position_;  // yaw = 135 degree

    bonus_right_.pose.orientation.z = 0.3826834;
    bonus_right_.pose.orientation.w = 0.9238795;

    bonus_left_.pose.orientation.z = 0.9238795;
    bonus_left_.pose.orientation.w = 0.3826834;
    chassis_cmd_sent_ = false;
    bonus_start_time_ = 0;
    sentry_ori_ = true;
  }

  void OnTerminate(BehaviorState state)
  {
    chassis_executor_->Cancel();
    chassis_cmd_sent_ = false;

    bonus_start_time_ = 0;

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
      chassis_executor_->Execute(bonus_position_);
    }
    else if (chassis_cmd_sent_ && executor_state == BehaviorState::FAILURE)
    {
      return BehaviorState::FAILURE;
    }
    else if (chassis_cmd_sent_ && executor_state != BehaviorState::RUNNING)
    {
      int bonus_status = blackboard_ptr_->GetBonusStatus();
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      auto linear_distance = blackboard_ptr_->GetDistance(robot_map_pose, bonus_position_);

      // ROS_INFO_STREAM_THROTTLE(1, "distance: " << linear_distance);
      // ROS_INFO_STREAM_THROTTLE(1, "Supply Status: " << bonus_status);

      if (linear_distance <= 0.1)
      {
        // else if (bonus_status == 0)
        // {
        //   ROS_INFO_STREAM("looking for bonux");
        //   blackboard_ptr_->MoveAround();
        // }
        if (bonus_status == 0)
          bonus_start_time_ = 0;
        if (bonus_status == 1)
        {
          ROS_INFO_STREAM("occuplying bonux");
          Sentry(bonus_position_, bonus_right_, bonus_left_, sentry_ori_);

          if (bonus_start_time_ == 0)
          {
            // 0 = unoccupied, 1 = being occupied, 2 = occupied
            bonus_start_time_ = ros::Time::now().toSec();

            ROS_INFO("START TIME %f", ros::Time::now().toSec());
          }
          else
          {
            ROS_INFO("OCCUPYINF TIME %f", ros::Time::now().toSec() - bonus_start_time_);
          }
        }
        else if (bonus_status == 2)
        {
          return BehaviorState::SUCCESS;
        }
        else if (ros::Time::now().toSec() - bonus_start_time_ > 30)
        {
          ROS_WARN("BONUS time out! LEAVE");
          return BehaviorState::FAILURE;
        }
      }
    }

    return BehaviorState::RUNNING;
  }

  ~BonusAction() = default;

private:
  //! executor
  const ChassisExecutor::Ptr chassis_executor_;

  std::string robot_name_;

  //! reload goal
  geometry_msgs::PoseStamped bonus_position_;
  geometry_msgs::PoseStamped bonus_right_;
  geometry_msgs::PoseStamped bonus_left_;

  bool chassis_cmd_sent_;
  double bonus_start_time_;
  bool sentry_ori_;

  void Sentry(const geometry_msgs::PoseStamped bonus_position, const geometry_msgs::PoseStamped bonus_right, const geometry_msgs::PoseStamped bonus_left, bool sentry_ori)
  {
    auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
    float target_diff = 0.08; // 5 degree
                              // tf::createQuaternionMsgFromRollPitchYaw(0, 0, );
    auto quat_diff = tf::createQuaternionMsgFromRollPitchYaw(0, 0, target_diff);

    auto target_pose = bonus_position;

    if (sentry_ori)
    {
      auto angle_diff = blackboard_ptr_->GetAngle(robot_map_pose, bonus_right);

      if (std::abs(angle_diff) > target_diff)
      {
        chassis_executor_->Execute(bonus_right);
      }
      else
      {
        chassis_executor_->Execute(bonus_left);
        sentry_ori = false;
      }
    }
    else
    {
      auto angle_diff = blackboard_ptr_->GetAngle(robot_map_pose, bonus_left);

      if (std::abs(angle_diff) > target_diff)
      {
        chassis_executor_->Execute(bonus_left);
      }
      else
      {
        chassis_executor_->Execute(bonus_right);
        sentry_ori = true;
      }
    }
  }
};

} // namespace roborts_decision
// namespace roborts_decision

#endif //ROBORTS_DECISION_bonus_ACTION_H

#ifndef ROBORTS_DECISION_ESCAPE_ACTION_H
#define ROBORTS_DECISION_ESCAPE_ACTION_H

#include "io/io.h"
#include <ros/ros.h>
#include "roborts_msgs/TwistAccel.h"

#include "blackboard/blackboard.h"
#include "behavior_tree/behavior_state.h"

namespace roborts_decision
{
class EscapeAction : public ActionNode
{
public:
  EscapeAction(const Blackboard::Ptr &blackboard) : ActionNode("action_escape", blackboard),
                                                    chassis_executor_(blackboard->chassis_executor)
  {
  }

  void OnInitialize()
  {
    // init whirl velocity
    whirl_vel_.accel.linear.x = 0;
    whirl_vel_.accel.linear.x = 0;
    whirl_vel_.accel.linear.y = 0;
    whirl_vel_.accel.linear.z = 0;

    whirl_vel_.accel.angular.x = 0;
    whirl_vel_.accel.angular.y = 0;
    whirl_vel_.accel.angular.z = 0;

    left_x_limit_ = blackboard_ptr_->decision_config.escape().left_x_limit();
    right_x_limit_ = blackboard_ptr_->decision_config.escape().right_x_limit();
    robot_x_limit_ = blackboard_ptr_->decision_config.escape().robot_x_limit();
    left_random_min_x_ = blackboard_ptr_->decision_config.escape().left_random_min_x();
    left_random_max_x_ = blackboard_ptr_->decision_config.escape().left_random_max_x();
    right_random_min_x_ = blackboard_ptr_->decision_config.escape().right_random_min_x();
    right_random_max_x_ = blackboard_ptr_->decision_config.escape().right_random_max_x();

    whirl_vel_.twist.angular.z = blackboard_ptr_->decision_config.whirl_vel().angle_z_vel();
    whirl_vel_.twist.angular.y = blackboard_ptr_->decision_config.whirl_vel().angle_y_vel();
    whirl_vel_.twist.angular.x = blackboard_ptr_->decision_config.whirl_vel().angle_x_vel();
  }

  BehaviorState Update()
  {
    auto executor_state = chassis_executor_->Update();

    if (executor_state != BehaviorState::RUNNING)
    {
      if (blackboard_ptr_->IsEnemyDetected())
      {

        geometry_msgs::PoseStamped enemy;
        enemy = blackboard_ptr_->GetEnemy();
        float goal_yaw, goal_x, goal_y;
        unsigned int goal_cell_x, goal_cell_y;
        unsigned int enemy_cell_x, enemy_cell_y;

        std::random_device rd;
        std::mt19937 gen(rd());

        auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
        float x_min, x_max;
        if (enemy.pose.position.x < left_x_limit_)
        {
          x_min = right_random_min_x_;
          x_max = right_random_max_x_;
        }
        else if (enemy.pose.position.x > right_x_limit_)
        {
          x_min = left_random_min_x_;
          x_max = left_random_max_x_;
        }
        else
        {
          if ((robot_x_limit_ - robot_map_pose.pose.position.x) >= 0)
          {
            x_min = left_random_min_x_;
            x_max = left_random_max_x_;
          }
          else
          {
            x_min = right_random_min_x_;
            x_max = right_random_max_x_;
          }
        }

        std::uniform_real_distribution<float> x_uni_dis(x_min, x_max);
        std::uniform_real_distribution<float> y_uni_dis(0, 5);
        //std::uniform_real_distribution<float> yaw_uni_dis(-M_PI, M_PI);

        auto get_enemy_cell = blackboard_ptr_->GetCostMap2D()->World2Map(enemy.pose.position.x,
                                                                         enemy.pose.position.y,
                                                                         enemy_cell_x,
                                                                         enemy_cell_y);

        if (!get_enemy_cell)
        {
          chassis_executor_->Execute(whirl_vel_);
          return BehaviorState::RUNNING;
        }

        while (true)
        {
          goal_x = x_uni_dis(gen);
          goal_y = y_uni_dis(gen);
          auto get_goal_cell = blackboard_ptr_->GetCostMap2D()->World2Map(goal_x,
                                                                          goal_y,
                                                                          goal_cell_x,
                                                                          goal_cell_y);

          if (!get_goal_cell)
          {
            continue;
          }

          auto index = blackboard_ptr_->GetCostMap2D()->GetIndex(goal_cell_x, goal_cell_y);
          //          costmap_2d_->GetCost(goal_cell_x, goal_cell_y);
          if (blackboard_ptr_->GetCharMap()[index] >= 253)
          {
            continue;
          }

          unsigned int obstacle_count = 0;
          for (FastLineIterator line(goal_cell_x, goal_cell_y, enemy_cell_x, enemy_cell_y); line.IsValid(); line.Advance())
          {
            auto point_cost = blackboard_ptr_->GetCostMap2D()->GetCost((unsigned int)(line.GetX()), (unsigned int)(line.GetY())); //current point's cost

            if (point_cost > 253)
            {
              obstacle_count++;
            }
          }

          if (obstacle_count > 5)
          { //TODO:  this should write in the proto file
            break;
          }
        }
        Eigen::Vector2d pose_to_enemy(enemy.pose.position.x - robot_map_pose.pose.position.x,
                                      enemy.pose.position.y - robot_map_pose.pose.position.y);
        goal_yaw = static_cast<float>(std::atan2(pose_to_enemy.coeffRef(1), pose_to_enemy.coeffRef(0)));
        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, goal_yaw);

        geometry_msgs::PoseStamped escape_goal;
        escape_goal.header.frame_id = "map";
        escape_goal.header.stamp = ros::Time::now();
        escape_goal.pose.position.x = goal_x;
        escape_goal.pose.position.y = goal_y;
        escape_goal.pose.orientation = quaternion;
        //return exploration_goal;
        chassis_executor_->Execute(escape_goal);
      }
      else
      {
        chassis_executor_->Execute(whirl_vel_);
        ROS_DEBUG("No Enemy Detected");
        return BehaviorState::SUCCESS;
      }
    }
    return BehaviorState::RUNNING;
  }

  void OnTerminate(BehaviorState state)
  {
    chassis_executor_->Cancel();
  }

  ~EscapeAction()
  {
  }

private:
  //! executor
  const ChassisExecutor::Ptr chassis_executor_;

  float left_x_limit_, right_x_limit_;
  float robot_x_limit_;
  float left_random_min_x_, left_random_max_x_;
  float right_random_min_x_, right_random_max_x_;

  //! whirl velocity
  //  geometry_msgs::Twist whirl_vel_;
  roborts_msgs::TwistAccel whirl_vel_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_ESCAPE_ACTION_H

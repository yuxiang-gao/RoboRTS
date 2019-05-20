#ifndef ROBORTS_DECISION_CHASE_ACTION_H
#define ROBORTS_DECISION_CHASE_ACTION_H

#include "io/io.h"
#include <ros/ros.h>
#include <ros/console.h>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"
#include "utils/line_iterator.h"

namespace roborts_decision
{
class ChaseAction : public ActionNode
{
public:
	ChaseAction(const ChassisExecutor::Ptr &chassis_executor,
				const Blackboard::Ptr &blackboard) : ActionNode("chase_action", blackboard),
													 chassis_executor_(chassis_executor)
	{
	}

	void OnInitialize()
	{
		chase_goal_.header.frame_id = "map";
		chase_goal_.pose.orientation.x = 0;
		chase_goal_.pose.orientation.y = 0;
		chase_goal_.pose.orientation.z = 0;
		chase_goal_.pose.orientation.w = 1;

		chase_goal_.pose.position.x = 0;
		chase_goal_.pose.position.y = 0;
		chase_goal_.pose.position.z = 0;

		chase_buffer_.resize(2);
		chase_count_ = 0;

		cancel_goal_ = true;
	}

	BehaviorState Update()
	{

		auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
		auto executor_state = chassis_executor_->Update();
		if (executor_state != BehaviorState::RUNNING)
		{
			chase_buffer_[chase_count_++ % 2] = blackboard_ptr_->GetEnemy();

			chase_count_ = chase_count_ % 2;

			auto dx = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose.pose.position.x;
			auto dy = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose.pose.position.y;
			auto yaw = std::atan2(dy, dx);

			if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0)
			{
				if (cancel_goal_)
				{
					chassis_executor_->Cancel();
					cancel_goal_ = false;
				}
				return BehaviorState::FAILURE;
			}
			else
			{
				auto orientation = tf::createQuaternionMsgFromYaw(yaw);
				geometry_msgs::PoseStamped reduce_goal;
				reduce_goal.pose.orientation = robot_map_pose.pose.orientation;

				reduce_goal.header.frame_id = "map";
				reduce_goal.header.stamp = ros::Time::now();
				reduce_goal.pose.position.x = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - 1.2 * cos(yaw);
				reduce_goal.pose.position.y = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - 1.2 * sin(yaw);
				auto enemy_x = reduce_goal.pose.position.x;
				auto enemy_y = reduce_goal.pose.position.y;
				reduce_goal.pose.position.z = 1;
				unsigned int goal_cell_x, goal_cell_y;

				// if necessary add mutex lock
				//blackboard_->GetCostMap2D()->GetMutex()->lock();
				auto get_enemy_cell = blackboard_ptr_->GetCostMap2D()->World2Map(enemy_x,
																				 enemy_y,
																				 goal_cell_x,
																				 goal_cell_y);
				//blackboard_->GetCostMap2D()->GetMutex()->unlock();

				if (!get_enemy_cell)
				{
					return BehaviorState::FAILURE;
				}

				auto robot_x = robot_map_pose.pose.position.x;
				auto robot_y = robot_map_pose.pose.position.y;
				unsigned int robot_cell_x, robot_cell_y;
				double goal_x, goal_y;
				blackboard_ptr_->GetCostMap2D()->World2Map(robot_x,
														   robot_y,
														   robot_cell_x,
														   robot_cell_y);

				if (blackboard_ptr_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253)
				{

					bool find_goal = false;
					for (FastLineIterator line(goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance())
					{

						auto point_cost = blackboard_ptr_->GetCostMap2D()->GetCost((unsigned int)(line.GetX()), (unsigned int)(line.GetY())); //current point's cost

						if (point_cost >= 253)
						{
							continue;
						}
						else
						{
							find_goal = true;
							blackboard_ptr_->GetCostMap2D()->Map2World((unsigned int)(line.GetX()),
																	   (unsigned int)(line.GetY()),
																	   goal_x,
																	   goal_y);

							reduce_goal.pose.position.x = goal_x;
							reduce_goal.pose.position.y = goal_y;
							break;
						}
					}
					if (find_goal)
					{
						cancel_goal_ = true;
						chassis_executor_->Execute(reduce_goal);
						return BehaviorState::SUCCESS;
					}
					else
					{
						if (cancel_goal_)
						{
							chassis_executor_->Cancel();
							cancel_goal_ = false;
						}
						return BehaviorState::FAILURE;
					}
				}
				else
				{
					cancel_goal_ = true;
					chassis_executor_->Execute(reduce_goal);
					return BehaviorState::FAILURE;
				}
			}
		}
		return BehaviorState::RUNNING;
	}

	void OnTerminate(BehaviorState state)
	{
		ROS_DEBUG_COND_NAMED(state == BehaviorState::FAILURE, GetName().c_str(), "action %s failed", GetName().c_str());
		ROS_DEBUG_COND_NAMED(state == BehaviorState::SUCCESS, GetName().c_str(), "action %s succeeded", GetName().c_str());
		chassis_executor_->Cancel();
	}

	void SetGoal(geometry_msgs::PoseStamped chase_goal)
	{
		chase_goal_ = chase_goal;
	}

	~ChaseAction() = default;

private:
	//! executor
	const ChassisExecutor::Ptr chassis_executor_;

	//! chase goal
	geometry_msgs::PoseStamped chase_goal_;

	//! chase buffer
	std::vector<geometry_msgs::PoseStamped> chase_buffer_;
	unsigned int chase_count_;

	//! cancel flag
	bool cancel_goal_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_CHASE_ACTION_H

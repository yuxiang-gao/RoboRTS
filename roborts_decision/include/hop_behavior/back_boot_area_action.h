#ifndef ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
#define ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"

namespace roborts_decision
{

class BackBootAreaAction : public ActionNode
{
public:
    BackBootAreaAction(const ChassisExecutor::Ptr &chassis_executor,
                       const Blackboard::Ptr &blackboard) : ActionNode("back_boot_area_action", blackboard),
                                                            chassis_executor_(chassis_executor)

    {
    }

    void OnInitialize()
    {
        boot_position_.header.frame_id = "/map";

        boot_position_.pose.position.x = blackboard_ptr_->decision_config.master_bot().start_position().x();
        boot_position_.pose.position.z = blackboard_ptr_->decision_config.master_bot().start_position().z();
        boot_position_.pose.position.y = blackboard_ptr_->decision_config.master_bot().start_position().y();

        auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(
            blackboard_ptr_->decision_config.master_bot().start_position().roll(),
            blackboard_ptr_->decision_config.master_bot().start_position().pitch(),
            blackboard_ptr_->decision_config.master_bot().start_position().yaw());
        boot_position_.pose.orientation = master_quaternion;
        ROS_INFO("quatw  %f", boot_position_.pose.orientation.w);
    }

    BehaviorState Update()
    {
        auto executor_state = chassis_executor_->Update();
        ROS_INFO_STREAM_THROTTLE(1, "Executor State: " << (int)executor_state);
        if (executor_state != BehaviorState::RUNNING)
        {
            auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
            auto linear_distance = blackboard_ptr_->GetDistance(robot_map_pose, boot_position_);
            auto angular_distance = blackboard_ptr_->GetAngle(robot_map_pose, boot_position_);

            if (linear_distance > 0.2 || angular_distance > 0.5)
            {
                chassis_executor_->Execute(boot_position_);
                return BehaviorState::RUNNING;
            }
            else
            {
                return BehaviorState::SUCCESS;
            }
        }
        return executor_state;
    }

    void OnTerminate(BehaviorState state)
    {
        chassis_executor_->Cancel();
    }

    ~BackBootAreaAction() = default;

private:
    //! executor
    const ChassisExecutor::Ptr chassis_executor_;
    //! boot position
    geometry_msgs::PoseStamped boot_position_;
    //! chase buffer
    std::vector<geometry_msgs::PoseStamped> chase_buffer_;
    unsigned int chase_count_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
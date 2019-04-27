#ifndef ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
#define ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "behaviour_tree/behaviour_tree.h"
#include "blackboard/blackboard.h"

namespace roborts_decision
{

class BackBootAreaAction : public ActionNode
{
  public:
    BackBootAreaAction(ChassisExecutor *&chassis_executor,
                       const Blackboard::Ptr &blackboard) : ActionNode("BackBootArea", blackboard),
                                                            chassis_executor_(chassis_executor)

    {
        boot_position_.header.frame_id = "map";

        boot_position_.pose.position.x = blackboard_ptr_->decision_config.master_bot().start_position().x();
        boot_position_.pose.position.z = blackboard_ptr_->decision_config.master_bot().start_position().z();
        boot_position_.pose.position.y = blackboard_ptr_->decision_config.master_bot().start_position().y();

        auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(
            blackboard_ptr_->decision_config.master_bot().start_position().roll(),
            blackboard_ptr_->decision_config.master_bot().start_position().pitch(),
            blackboard_ptr_->decision_config.master_bot().start_position().yaw());
        boot_position_.pose.orientation = master_quaternion;
    }

    void OnInitialize()
    {

        auto robot_map_pose = blackboard_->GetRobotMapPose();
        auto dx = boot_position_.pose.position.x - robot_map_pose.pose.position.x;
        auto dy = boot_position_.pose.position.y - robot_map_pose.pose.position.y;

        auto boot_yaw = tf::getYaw(boot_position_.pose.orientation);
        auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(boot_position_.pose.orientation, rot1);
        tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
        auto d_yaw = rot1.angleShortestPath(rot2);

        if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
        {
            chassis_executor_->Execute(boot_position_);
        }
    }
    BehaviorState Run()
    {
        auto executor_state = Update();
        behavior_state_ = ActionNode::Run();
        return behavior_state_;
    }
    void OnTerminate(BehaviorState state)
    {
        chassis_executor_->Cancel();
    }

    BehaviorState Update()
    {
        return chassis_executor_->Update();
    }

    ~BackBootAreaBehavior() = default;

  private:
    //! executor
    ChassisExecutor *const chassis_executor_;
    //! boot position
    geometry_msgs::PoseStamped boot_position_;
    //! chase buffer
    std::vector<geometry_msgs::PoseStamped> chase_buffer_;
    unsigned int chase_count_;
}
} // namespace roborts_decision

#endif //ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
#ifndef ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
#define ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
#include <ros/ros.h>

#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"

namespace roborts_decision
{

class BootAreaBackupAction : public ActionNode
{
public:
    BootAreaBackupAction(const Blackboard::Ptr &blackboard) : ActionNode("action_boot_backup", blackboard),
                                                              chassis_executor_(blackboard->chassis_executor)

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

        ros::NodeHandle n;
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.linear.y = -0.1;
        vel_msg.linear.z = 0;
        // ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1000);
        ROS_INFO("quatw  %f", boot_position_.pose.orientation.w);
    }

    BehaviorState Update()
    {
        auto executor_state = chassis_executor_->Update();
        ROS_INFO_STREAM_THROTTLE(1, "Executor State: " << (int)executor_state);
        if (executor_state != BehaviorState::RUNNING)
        {
            auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
            // auto linear_distance = blackboard_ptr_->GetDistance(robot_map_pose, boot_position_);
            // auto angular_distance = blackboard_ptr_->GetAngle(robot_map_pose, boot_position_);
            auto robot_pose_x = robot_map_pose.pose.position.x;
            auto robot_pose_y = robot_map_pose.pose.position.y;
            auto robot_yaw = tf::getYaw(obot_map_pose.orientation) * 180 / 3.1415926;
            if (robot_yaw < 0)
                robot_yaw += 360;

            if (boot_position_.pose.position.x > 7.5 && boot_position_.pose.position.y < 0.5) // robot0
            {
                if (robot_yaw < 110 && robot_yaw > 250) // if robot is facing the wall
                {
                    vel_pub.publish(vel_msg);
                    return BehaviorState::RUNNING;
                }
                else
                    return BehaviorState::SUCCESS;
            }
            else if (boot_position_.pose.position.x < 0.5 && boot_position_.pose.position.y < 0.5) // robot1
            {
                if (robot_yaw > 150 && robot_yaw < 200) // if robot is facing the wall
                {
                    vel_pub.publish(vel_msg);
                    return BehaviorState::RUNNING;
                }
                else
                    return BehaviorState::SUCCESS;
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
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_BACK_BOOT_AREA_ACTION_H
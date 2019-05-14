#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"

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

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_test_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    auto chassis_executor = new roborts_decision::ChassisExecutor;
    auto blackboard = new roborts_decision::Blackboard(full_path);

    SelectorNode root_node("root_node", blackboard);
    root_node.SetLevel = 0;
    root_node.SetParent(NULL);
    PreconditionNode game_start("start", precondition_function = gameStart());
    SequenceNode sq_1("sq_1", blackboard);
    game_start.AddChildren(sq_1);
    sq_1.AddChildren({std::make_shared<BackBootAreaAction>(chassis_executor, blackboard)});

    return 0;
}
} // namespace roborts_decision
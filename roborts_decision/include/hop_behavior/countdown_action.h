#ifndef ROBORTS_DECISION_INIT_ACTION_H
#define ROBORTS_DECISION_INIT_ACTION_H

#include <ros/ros.h>

#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"

namespace roborts_decision
{
class CountdownAction : public ActionNode
{
public:
    CountdownAction(const Blackboard::Ptr &blackboard) : ActionNode("action_countdown", blackboard) {}

    void OnInitialize()
    {
        ROS_INFO("Initiate, turnon fric wheel");
        blackboard_ptr_->TurnOnFricWheel();
    }
    BehaviorState Update()
    {
        return BehaviorState::SUCCESS;
    }
    void OnTerminate(BehaviorState state)
    {
        return;
    }
};
} // namespace roborts_decision

#endif

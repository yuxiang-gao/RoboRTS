#ifndef ROBORTS_DECISION_SHOOT_ACTION_H
#define ROBORTS_DECISION_SHOOT_ACTION_H
#include <chrono>
#include "io/io.h"
#include <ros/ros.h>

#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"
#include "utils/line_iterator.h"

namespace roborts_decision
{
class ShootAction : public ActionNode
{
public:
    ShootAction(const Blackboard::Ptr &blackboard) : ActionNode("action_shoot", blackboard) {}
    void OnInitialize()
    {
    }

    void OnTerminate(BehaviorState state)
    {
    }

    BehaviorState Update()
    {
        blackboard_ptr_->Shoot(5);
        return BehaviorState::SUCCESS;
    }
};
} // namespace roborts_decision

#endif
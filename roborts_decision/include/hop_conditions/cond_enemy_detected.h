#ifndef COND_ENEMY_DETECTED_H
#define COND_ENEMY_DETECTED_H
#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"

namespace roborts_decision
{
// Return True if enemy is detected
class EnemyDetectedCondition : public PreconditionNode
{
public:
    EnemyDetectedCondition(const ChassisExecutor::Ptr &chassis_executor,
                           Blackboard::Ptr &blackboard) : PreconditionNode("enemy_detected_cond", blackboard), blackboard->IsEnemyDetected(), AbortType::SELF)
    {
    }
    ~EnemyDetectedCondition() = default;

    bool Reevaluation()
    {
        return Precondition();
    }
};
} // namespace roborts_decision
#endif
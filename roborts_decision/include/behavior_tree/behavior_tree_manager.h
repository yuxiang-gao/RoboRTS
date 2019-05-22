#ifndef ROBORTS_DECISION_BEHAVIOR_TREE_MANAGER_H
#define ROBORTS_DECISION_BEHAVIOR_TREE_MANAGER_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "behavior_tree/behavior_tree.h"
#include "executor/chassis_executor.h"
#include "blackboard/blackboard.h"

#include "hop_behavior/back_boot_area_action.h"
#include "hop_behavior/escape_action.h"
#include "hop_behavior/chase_action.h"
#include "hop_behavior/search_action.h"
#include "hop_behavior/patrol_action.h"
#include "hop_behavior/goal_action.h"
#include "hop_behavior/reload_action.h"
#include "hop_behavior/bonus_action.h"
#include "hop_behavior/countdown_action.h"
#include "hop_behavior/shoot_action.h"

namespace roborts_decision
{
class BehaviorTreeManager
{
public:
    typedef std::shared_ptr<BehaviorTreeManager> Ptr;
    BehaviorTreeManager(const Blackboard::Ptr &blackboard_ptr) : blackboard_ptr_(blackboard_ptr),
                                                                 chassis_executor_(blackboard_ptr_->chassis_executor),
                                                                 conditions_(),
                                                                 actions_(),
                                                                 composites_()
    {
        InitActions();
        InitConditions();
    }

    ~BehaviorTreeManager() = default;

    void InitActions()
    {
        actions_["action_back_boot_area"] = std::make_shared<BackBootAreaAction>(blackboard_ptr_); //Done
        actions_["action_chase"] = std::make_shared<ChaseAction>(blackboard_ptr_);
        actions_["action_search"] = std::make_shared<SearchAction>(blackboard_ptr_);
        actions_["action_escape"] = std::make_shared<EscapeAction>(blackboard_ptr_);
        actions_["action_patrol"] = std::make_shared<PatrolAction>(blackboard_ptr_);       //Done
        actions_["action_goal"] = std::make_shared<GoalAction>(blackboard_ptr_);           //Done
        actions_["action_reload"] = std::make_shared<ReloadAction>(blackboard_ptr_);       //Done
        actions_["action_bonus"] = std::make_shared<BonusAction>(blackboard_ptr_);         //TODO
        actions_["action_countdown"] = std::make_shared<CountdownAction>(blackboard_ptr_); //Done
        actions_["action_shoot"] = std::make_shared<ShootAction>(blackboard_ptr_);
    }

    void InitConditions()
    {
        AddCondition("condition_countdown", [=]() { return blackboard_ptr_->IsFiveSecondCD(); }, AbortType::LOW_PRIORITY);
        AddCondition("condition_game_start", [=]() { return blackboard_ptr_->IsGameStart(); }, AbortType::BOTH);
        AddCondition("condition_bonus", [=]() { return blackboard_ptr_->IsBonusAvailable(); }, AbortType::BOTH);
        AddCondition("condition_enemy_detected", [=]() { return blackboard_ptr_->IsEnemyDetected(); }, AbortType::BOTH);
        AddCondition("condition_reload", [=]() { return (blackboard_ptr_->IsNeedReload() && blackboard_ptr_->IsReloadAvailable()); }, AbortType::BOTH);
    }

    void AddCondition(std::string name,
                      std::function<bool()> precondition_function = std::function<bool()>(),
                      AbortType abort_type = AbortType::NONE)
    {
        conditions_[name] = std::make_shared<PreconditionNode>(name, blackboard_ptr_, precondition_function, abort_type);
        ROS_INFO("Add condition node %s", name.c_str());
    }

    void AddCompositeNode(std::string name)
    {
        if (name.find("sequence") != std::string::npos)
            AddSequenceNode(name);
        else if (name.find("selector") != std::string::npos)
            AddSelectorNode(name);
        else if (name.find("parallel") != std::string::npos)
            AddParallelNode(name);
    }

    void AddCompositeNodes(std::initializer_list<std::string> name_list)
    {
        for (auto name : name_list)
        {
            AddCompositeNode(name);
            ROS_INFO("Add node %s", name.c_str());
        }
    }

    void AddSequenceNode(std::string name)
    {
        composites_[name] = std::make_shared<SequenceNode>(name, blackboard_ptr_);
        ROS_INFO("Add node %s", name.c_str());
    }

    void AddSelectorNode(std::string name)
    {
        composites_[name] = std::make_shared<SelectorNode>(name, blackboard_ptr_);
        ROS_INFO("Add node %s", name.c_str());
    }

    void AddParallelNode(std::string name, int threshold = 1)
    {
        composites_[name] = std::make_shared<ParallelNode>(name, blackboard_ptr_, threshold);
        ROS_INFO("Add node %s", name.c_str());
    }

    DecoratorNode::Ptr GetConditionNode(std::string name)
    {
        DecoratorMap::const_iterator iter = conditions_.find(name);

        if (iter != conditions_.end())
        {
            return iter->second;
        }
        else
        {
            ROS_ERROR("No decorator node by the name of %s", name.c_str());
        }
        return nullptr;
    }

    ActionNode::Ptr GetActionNode(std::string name)
    {
        ActionMap::const_iterator iter = actions_.find(name);

        if (iter != actions_.end())
        {
            return iter->second;
        }
        else
        {
            ROS_ERROR("No action node by the name of %s", name.c_str());
        }
        return nullptr;
    }

    CompositeNode::Ptr GetCompositeNode(std::string name)
    {
        CompositeMap::const_iterator iter = composites_.find(name);

        if (iter != composites_.end())
        {
            return iter->second;
        }
        else
        {
            ROS_ERROR("No composite node by the name of %s", name.c_str());
            return nullptr;
        }
    }

    BehaviorNode::Ptr GetNodeByName(std::string name)
    {
        if (name.find("action") != std::string::npos)
            return GetActionNode(name);
        else if (name.find("condition") != std::string::npos)
            return GetConditionNode(name);
        else if (name.find("sequence") != std::string::npos || name.find("selector") != std::string::npos || name.find("parallel") != std::string::npos)
            return GetCompositeNode(name);
        ROS_ERROR("Can't find node named %s", name.c_str());
        return nullptr;
    }

    BehaviorType ParseName(std::string name)
    {
        if (name.find("action") != std::string::npos)
            return BehaviorType::ACTION;
        else if (name.find("condition") != std::string::npos)
            return BehaviorType::PRECONDITION;
        else if (name.find("sequence") != std::string::npos)
            return BehaviorType::SEQUENCE;
        else if (name.find("selector") != std::string::npos)
            return BehaviorType::SELECTOR;
        else if (name.find("parallel") != std::string::npos)
            return BehaviorType::PARALLEL;
    }

    void ConnectDecorator(const DecoratorNode::Ptr &parent, const BehaviorNode::Ptr &child)
    {
        parent->SetChild(child);
    }

    void ConnectComposite(const CompositeNode::Ptr &parent, std::initializer_list<BehaviorNode::Ptr> children_list)
    {
        parent->AddChildren(children_list);
    }

    void ConnectComposite(const CompositeNode::Ptr &parent, const BehaviorNode::Ptr &children)
    {
        parent->AddChildren(children);
    }

    void Connect(std::string parent_name, std::string child_name)
    {
        ROS_INFO("Connect node %s and %s", parent_name.c_str(), child_name.c_str());
        if (parent_name.find("condition") != std::string::npos)
        {
            ConnectDecorator(GetConditionNode(parent_name), GetNodeByName(child_name));
        }
        else if (parent_name.find("sequence") != std::string::npos || parent_name.find("selector") != std::string::npos || parent_name.find("parallel") != std::string::npos)
        {
            ConnectComposite(GetCompositeNode(parent_name), GetNodeByName(child_name));
        }
        else if (parent_name.find("action") != std::string::npos)
            ROS_ERROR("Action Nodes cannot be parent.");
    }

    void Connect(std::string parent_name, std::initializer_list<std::string> children_name_list)
    {
        for (auto &child_name : children_name_list)
            ROS_INFO("Connect node %s and %s", parent_name.c_str(), child_name.c_str());
        if (parent_name.find("condition") != std::string::npos)
        {
            if (children_name_list.size() == 1)
                Connect(parent_name, *children_name_list.begin());
            else
                ROS_ERROR("Decorator Nodes cannot have multiple.");
            return;
        }
        else if (parent_name.find("sequence") != std::string::npos || parent_name.find("selector") != std::string::npos || parent_name.find("parallel") != std::string::npos)
        {
            for (auto &child_name : children_name_list)
                Connect(parent_name, child_name);
        }
        else if (parent_name.find("action") != std::string::npos)
            ROS_ERROR("Action Nodes cannot be parent.");
    }

    void Run(std::string root_name, int cycle = 100)
    {
        BehaviorTree behavior_tree(GetNodeByName(root_name), 100);
        behavior_tree.Run();
    }

private:
    typedef std::map<std::string, DecoratorNode::Ptr> DecoratorMap;
    typedef std::map<std::string, ActionNode::Ptr> ActionMap;
    typedef std::map<std::string, CompositeNode::Ptr> CompositeMap;
    const Blackboard::Ptr blackboard_ptr_;
    const ChassisExecutor::Ptr chassis_executor_;
    DecoratorMap conditions_;
    ActionMap actions_;
    CompositeMap composites_;
};
} // namespace roborts_decision

#endif
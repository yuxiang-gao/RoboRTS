#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "behaviour_tree/behaviour_tree.h"
#include "executor/chassis_executor.h"
#include "blackboard/blackboard.h"

#include "hop_behaviour/back_boot_area_action.h"
#include "hop_behaviour/escape_action.h"
#include "hop_behaviour/chase_action.h"
#include "hop_behaviour/search_action.h"
#include "hop_behaviour/patrol_action.h"
#include "hop_behaviour/goal_action.h"
namespace roborts_decision
{

// class HopTree
// {
// public:
//     HopTree(const std::string &proto_file_path) :
//         blackboard_ptr_(boost::make_shared<Blackboard>(full_path),
//         chassis_executor_(boost::make_shared<ChassisExecutor>()),
//         back_boot_area_action_(chassis_executor_, blackboard_ptr_),
//         chase_action_(chassis_executor_, blackboard_ptr_),
//         search_action_(chassis_executor_, blackboard_ptr_),
//         escape_action_(chassis_executor_, blackboard_ptr_),
//         patrol_action_(chassis_executor_, blackboard_ptr_),
//         goal_action_(chassis_executor_, blackboard_ptr_)
//     {

//     }

// private:
//     Blackboard::ConstPtr blackboard_ptr_;
//     ChassisExecutor::ConstPtr chassis_executor_;

//     roborts_decision::BackBootAreaAction back_boot_area_action_;
//     roborts_decision::ChaseAction chase_action_;
//     roborts_decision::SearchAction search_action_;
//     roborts_decision::EscapeAction escape_action_;
//     roborts_decision::PatrolAction patrol_action_;
//     roborts_decision::GoalAction goal_action_;
// };

bool localizeRight(){
	
	//returns true if robot is at right of arena at start of round	
	
	return false;
}

bool localizeLeft(){
	
	//returns true if robot is at left of arena at start of round	
	
	return false;
}

bool startOfRound(){
	//returns true if within 5 secs of start of round and neither of robot is in DZ zone
	
	return false;
}

bool startOfRoundAndDZ(){
	//returns true if within 10 secs of start of round and one of the robots has activated DZ
	
	return false;
}

bool ifDZ_R2(){
	// returns true if 1_2_min_mark && (((R12_engage || !R12_engage) && R1_HP>=R2_HP) || (R1_engage & !R2_engage))
	

	return false;
	
	
bool ifDZ_R1(){
	// returns true if 1_2_min_mark && (((R12_engage || !R12_engage) && R1_HP<R2_HP) || (!R1_engage & R2_engage))
	

	return false;

bool ifengage(){
	// returns true if goodHP && is_ammo && is_enenmy
	
	
	return false;
}

bool ifpatrol(){
	// returns true if goodHP && is_ammo && !is_enenmy
	
	
	return false;
}

bool ifreload(){
	// returns true if goodHP && !is_ammo
	
	return false;
}

bool ifhide(){
	// returns if !goodHP
	
	return false;
}


class DTPreconditions
{
public:
    DTPreconditions(ChassisExecutor::Ptr &chasis_executor, Blackboard::Ptr &blackboard) : chasis_executor_(chasis_executor),
                                                                                          blackboard_(blackboard)
    {
    }
    ~DTPreconditions() = default;

private:
    ChassisExecutor::ConstPtr chasis_executor_;
    Blackboard::ConstPtr blackboard_;
};
} // namespace roborts_decision

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_test_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    auto chassis_executor = new roborts_decision::ChassisExecutor();
    auto blackboard = new roborts_decision::Blackboard(full_path);

    roborts_decision::BackBootAreaAction back_boot_area_action(chassis_executor, blackboard);
    roborts_decision::ChaseAction chase_action(chassis_executor, blackboard);
    roborts_decision::SearchAction search_action(chassis_executor, blackboard);
    roborts_decision::EscapeAction escape_action(chassis_executor, blackboard);
    roborts_decision::PatrolAction patrol_action(chassis_executor, blackboard);
    roborts_decision::GoalAction goal_action(chassis_executor, blackboard);
    roborts_decision::ReloadAction reload_action("robot_1", chassis_executor, blackboard)
    roborts_decision::DZAction DZ_action(chassis_executor, blackboard)

    // SelectorNode root_node("root_node", blackboard);
    // root_node.SetLevel(0);
    // root_node.SetParent(NULL);
    // PreconditionNode game_start("start", precondition_function = gameStart());
    // SequenceNode sq_1("sq_1", blackboard);
    // game_start.AddChildren(sq_1);
    // sq_1.AddChildren({std::make_shared<BackBootAreaAction>(chassis_executor, blackboard)});
    
    
    PreconditionNode ammo_right("AMR", blackboard,
                   precondition_function = localizeRight(), //true if ammo robot R1 at right of arena
                   abort_type = AbortType::NONE);
    PreconditionNode ammo_left("AML", blackboard,
                   precondition_function = localizeLeft(), //true if ammo robot R1 at left of arena
                   abort_type = AbortType::NONE);
  //AMR children
    SequenceNode Sq1(Sq1, blackboard);
    AMR.AddChildren(Sq1);
  
  //Sq1 children
    PreconditionNode DZ_hide("DZ_hide", blackboard,
                        precondition_function = startOfRound(), // true if start of round
                        abort_type = AbortType::NONE); 
    DZ_hide.AddChildren(escape_action); // Goal would be defense zone R1 to defense zone and R2 hides
    PreconditionNode patrol_reload("patrol_reload", blackboard,
                       precondition_function = startOfRoundAndDZ() // true if start of round and DZ is activated by one of the robots
                       abort_type = AbortType::NONE);
    patrol_reload.AddChildren(reload_action); // goal would be reloading zone. R1 patrols and R2 to reload

    SelectorNode DZ("DZ", blackboard);
    SelectorNode combat('engage', blackboard);
    Sq1.AddChildren({ DZ_hide, patrol_reload,DZ,combat})
  
  //DZ children
  
    PreconditionNode DZ_R2("DZ_R2", blackboard,
                       precondition_function = ifDZ_R2(), // true if 1_2_min_mark && (((R12_engage || !R12_engage) && R1_HP>=R2_HP) || (R1_engage & !R2_engage))
                       abort_type = AbortType::NONE);
    DZ_R2.AddChildren(DZ_action); // goal would be defense zone. move R2 to DZ (modify goal_behavior)
  
   /* PreconditionNode DZ_R1("DZ_R1", blackboard,
                       precondition_function = ifDZ_R1(), // true if 1_2_min_mark && (((R12_engage || !R12_engage) && R1_HP<R2_HP) || (!R1_engage & R2_engage))
                       abort_type = AbortType::NONE);
    DZ_R1.AddChildren(DZ_action); // goal would be Defense zone. move R1 to DZ (modify goal_behavior)
  */
    DZ.AddChildren(DZ_R2);
  
  // combat children Do for both robots
    PreconditionNode engage("engage", blackboard,
                       precondition_function = ifengage(), // true if goodHP && is_ammo && is_enenmy
                       abort_type = AbortType::NONE);
  
    engage.AddChildren(chase_action); // chase the enemy
  
    PreconditionNode patrol("patrol", blackboard,
                       precondition_function = ifpatrol(), // true if goodHP && is_ammo && !is_enenmy
                       abort_type = AbortType::NONE);
  
    patrol.AddChildren(patrol_action); // patrol
  
    PreconditionNode reload("reload", blackboard,
                       precondition_function = ifreload(), // true if goodHP && !is_ammo
                       abort_type = AbortType::NONE);
  
    reload.AddChildren(DZ_action); // change goal to DZ move robot to DZ, update new_goal
  
    PreconditionNode hide("hide", blackboard,
                       precondition_function = ifhide(), // true if !goodHP
                       abort_type = AbortType::NONE);
  
    hide.AddChildren(escape_action); // may have to modify
  
    combat.AddChildren({engage, patrol, reload, hide})
  
  
  //AML children
    SequenceNode Sq2(Sq2, blackboard);
    AML.AddChildren(Sq2);
  
  //Sq2 children
    PreconditionNode patrol_DZ("patrol_DZ", blackboard,
                       precondition_function = startOfRound(), // true if start of round
                       abort_type = AbortType::NONE); 
    patrol_DZ.AddChildren(DZ_action); //  goal would be Defense zone. R1 patrols and R2 to DZ

    PreconditionNode engage_reload("engage_reload", blackboard,
                       precondition_function = startOfRoundAndDz() // true if start of round and activated by one of the robots
                       abort_type = AbortType::NONE);
    engage_reload.AddChildren(reload_action); // goal to be reloading zone. R1 engages and R2 to reload

    Sq2.AddChildren({ patrol_DZ, engage_reload,DZ,combat})


    //PreconditionNode cond_enemy_detect("cond_enemy_detect", blackboard, blackboard->IsEnemyDetected(), AbortType::SELF);

    return 0;
}

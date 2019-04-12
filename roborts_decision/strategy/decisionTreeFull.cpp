#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"

#include "behaviour_tree/behaviour_tree.h"
#include "../blackboard/blackboard.h"


//pre-condition functions 
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

int main(int argc, char **argv) { // not sure about the arguments
  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);

 /*  SequenceNode root_node('root_node', blackboard);
  rootNode.SetLevel=0;
  rootNode.SetParent(NULL);
  rootNode.AddChildren(roborts_decision::GoalBehavior);
  rootNode.AddChildren(roborts_decision::PatrolBehavior); */
  
  
  SelectorNode root_node('root_node', blackboard);
  rootNode.SetLevel=0;
  rootNode.SetParent(NULL);
  // root children
  PreconditionNode ammo_right('AMR', blackboard,
                   precondition_function = localizeRight(), //true if ammo robot R1 at right of arena
                   abort_type = AbortType::NONE);
  PreconditionNode ammo_left('AML', blackboard,
                   precondition_function = localizeLeft(), //true if ammo robot R1 at left of arena
                   abort_type = AbortType::NONE);
  //AMR children
  SequenceNode Sq1(Sq1, blackboard);
  AMR.AddChildren(Sq1);
  
  //Sq1 children
  PreconditionNode DZ_hide('DZ_hide', blackboard,
                   precondition_function = startOfRound(), // true if start of round
                   abort_type = AbortType::NONE); 
  DZ_hide.AddChildren({roborts_decision::GoalBehavior, roborts_decision::escape_behavior }); // R1 to defense zone and R2 hides
  PreconditionNode patrol_reload('patrol_reload', blackboard,
                   precondition_function = startOfRoundAndDZ() // true if start of round and DZ is activated by one of the robots
                   abort_type = AbortType::NONE);
  patrol_reload.AddChildren({roborts_decision::PatrolBehavior,roborts_decision::GoalBehavior}); // R1 patrols and R2 to reload

  SelectorNode DZ('DZ', blackboard);
  SelectorNode combat('engage', blackboard);
  Sq1.AddChildren({ DZ_hide, patrol_reload,DZ,combat})
  
  //DZ children
  
  PreconditionNode DZ_R2('DZ_R2', blackboard,
                   precondition_function = ifDZ_R2(), // true if 1_2_min_mark && (((R12_engage || !R12_engage) && R1_HP>=R2_HP) || (R1_engage & !R2_engage))
                   abort_type = AbortType::NONE);
  DZ_R2.AddChildren(roborts_decision::GoalBehavior); // move R2 to DZ (modify goal_behavior)
  
  PreconditionNode DZ_R1('DZ_R1', blackboard,
                   precondition_function = ifDZ_R1(), // true if 1_2_min_mark && (((R12_engage || !R12_engage) && R1_HP<R2_HP) || (!R1_engage & R2_engage))
                   abort_type = AbortType::NONE);
  DZ_R1.AddChildren(roborts_decision::GoalBehavior); // move R1 to DZ (modify goal_behavior)
  
  DZ.AddChildren({DZ_R1, DZ_R2});
  
  // combat children Do for both robots
  PreconditionNode engage('engage', blackboard,
                   precondition_function = ifengage(), // true if goodHP && is_ammo && is_enenmy
                   abort_type = AbortType::NONE);
  engage.AddChildren(roborts_decision::ChaseBehavior); // chase the enemy
  
  PreconditionNode patrol('patrol', blackboard,
                   precondition_function = ifpatrol(), // true if goodHP && is_ammo && !is_enenmy
                   abort_type = AbortType::NONE);
  patrol.AddChildren(roborts_decision::PatrolBehavior); // patrol
  
  PreconditionNode reload('reload', blackboard,
                   precondition_function = ifreload(), // true if goodHP && !is_ammo
                   abort_type = AbortType::NONE);
  reload.AddChildren(roborts_decision::GoalBehavior); // move robot to DZ, update new_goal
  
  PreconditionNode hide('hide', blackboard,
                   precondition_function = ifhide(), // true if !goodHP
                   abort_type = AbortType::NONE);
  hide.AddChildren(roborts_decision::EscapeBehavior); // may have to modify
  
  combat.AddChildren({engage, patrol, reload, hide})
  
  
  //AML children
  SequenceNode Sq2(Sq2, blackboard);
  AML.AddChildren(Sq2);
  
  //Sq2 children
  PreconditionNode patrol_DZ('patrol_DZ', blackboard,
                   precondition_function = startOfRound(), // true if start of round
                   abort_type = AbortType::NONE); 
  patrol_DZ.AddChildren({roborts_decision::PatrolBehavior,roborts_decision::GoalBehavior}); // R1 patrols and R2 to DZ

  PreconditionNode engage_reload('engage_reload', blackboard,
                   precondition_function = startOfRoundAndDz() // true if start of round and activated by one of the robots
                   abort_type = AbortType::NONE);
  engage_reload.AddChildren({roborts_decision::ChaseBehavior,roborts_decision::GoalBehavior}); // R1 engages and R2 to reload

  Sq2.AddChildren({ patrol_DZ, engage_reload,DZ,combat})
    
	
	
	
  ros::init(argc, argv, "decisionTreeFull");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  
   BehaviorTree AItree(BehaviorNode::Ptr root_node, int cycle_duration);
   AItree.run();

  return 0;
}


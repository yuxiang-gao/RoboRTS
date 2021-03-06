#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "hop_behavior/back_boot_area_action.h"
#include "hop_behavior/escape_action.h"
#include "hop_behavior/chase_action.h"
#include "hop_behavior/search_action.h"
#include "hop_behavior/patrol_action.h"
#include "hop_behavior/goal_action.h"
#include "hop_behavior/reload_action.h"

void Command();
char command = '0';

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hop_behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto blackboard = std::make_shared<roborts_decision::Blackboard>(full_path);

  roborts_decision::BackBootAreaAction back_boot_area_action(blackboard);
  roborts_decision::ChaseAction chase_action(blackboard);
  roborts_decision::SearchAction search_action(blackboard);
  roborts_decision::EscapeAction escape_action(blackboard);
  roborts_decision::PatrolAction patrol_action(blackboard);
  roborts_decision::GoalAction goal_action(blackboard);
  roborts_decision::ReloadAction reload_action(blackboard);

  auto act_lambda = [](auto act) { ROS_INFO_STREAM_COND((int)act.Run() == 1, act.GetName().c_str() << " SUCCESS"); };
  // act_lambda(reload_action);
  // ros::spin();
  auto command_thread = std::thread(Command);
  ros::Rate rate(10);
  roborts_decision::BehaviorState state;
  while (ros::ok())
  {
    ros::spinOnce();
    switch (command)
    {
    //back to boot area
    case '1':
      state = back_boot_area_action.Run();
      break;
      //patrol
    case '2':
      state = patrol_action.Run();
      break;
      //chase.
    case '3':
      state = chase_action.Run();
      break;
      //search
    case '4':
      state = search_action.Run();
      break;
      //escape.
    case '5':
      state = escape_action.Run();
      break;
      //goal.
    case '6':
      state = goal_action.Run();
      break;
    case '7':
      state = reload_action.Run();
      break;
    case 27:
      if (command_thread.joinable())
      {
        command_thread.join();
      }
      return 0;
    default:
      break;
    }
    if (state == roborts_decision::BehaviorState::FAILURE || state == roborts_decision::BehaviorState::SUCCESS)
    {
      ROS_INFO("End state %d", (int)state);
      return 0;
    }
    rate.sleep();
  }

  return 0;
}

void Command()
{

  while (command != 27)
  {
    // std::cout << "**************************************************************************************" << std::endl;
    // std::cout << "*********************************please send a command********************************" << std::endl;
    // std::cout << "1: back boot area behavior" << std::endl
    //           << "2: patrol behavior" << std::endl
    //           << "3: chase_behavior" << std::endl
    //           << "4: search behavior" << std::endl
    //           << "5: escape behavior" << std::endl
    //           << "6: goal behavior" << std::endl
    //           << "esc: exit program" << std::endl;
    // std::cout << "**************************************************************************************" << std::endl;
    // std::cout << "> ";
    std::cin >> command;
    // if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != 27)
    // {
    //   std::cout << "please input again!" << std::endl;
    //   std::cout << "> ";
    //   std::cin >> command;
    // }
  }
}

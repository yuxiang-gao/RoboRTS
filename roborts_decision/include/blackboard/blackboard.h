/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <string>

#include "roborts_msgs/ArmorDetectionAction.h"
//Referee System
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/SupplierStatus.h"

#include "io/io.h"
#include "../../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

namespace roborts_decision
{
//! structure to store msg from referee system
class RefereeSystemInfo
{
public:
  roborts_msgs::GameStatusPtr game_status;
  roborts_msgs::GameResultPtr game_result;
  roborts_msgs::GameSurvivorPtr game_survivor;
  roborts_msgs::BonusStatusPtr bonus_status;
  roborts_msgs::SupplierStatusPtr supplier_status;
  roborts_msgs::RobotStatusPtr robot_status;
  roborts_msgs::RobotHeatPtr robot_heat;
  roborts_msgs::RobotBonusPtr robot_bonus;
  roborts_msgs::RobotDamagePtr robot_damage;
  roborts_msgs::RobotShootPtr robot_shoot;
  RefereeSystemInfo() : game_status(boost::make_shared<roborts_msgs::GameStatus>()),
                        game_result(boost::make_shared<roborts_msgs::GameResult>()),
                        game_survivor(boost::make_shared<roborts_msgs::GameSurvivor>()),
                        bonus_status(boost::make_shared<roborts_msgs::BonusStatus>()),
                        supplier_status(boost::make_shared<roborts_msgs::SupplierStatus>()),
                        robot_status(boost::make_shared<roborts_msgs::RobotStatus>()),
                        robot_heat(boost::make_shared<roborts_msgs::RobotHeat>()),
                        robot_bonus(boost::make_shared<roborts_msgs::RobotBonus>()),
                        robot_damage(boost::make_shared<roborts_msgs::RobotDamage>()),
                        robot_shoot(boost::make_shared<roborts_msgs::RobotShoot>())
  {
  }
  ~RefereeSystemInfo()
  {
    game_status.reset();
    game_result.reset();
    game_survivor.reset();
    bonus_status.reset();
    supplier_status.reset();
    robot_status.reset();
    robot_heat.reset();
    robot_bonus.reset();
    robot_damage.reset();
    robot_shoot.reset();
  }
};

class Blackboard
{
public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef std::shared_ptr<Blackboard coonst> ConstPtr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  // config
  roborts_decision::DecisionConfig decision_config;
  // dictionary, used to store referee info for each robot
  std::map<std::string /*robot_name*/, boost::shared_ptr<RefereeSystemInfo>> referee_info;
  // RefereeSystemInfo wing_status;
  // RefereeSystemInfo master_status;
  explicit Blackboard(const std::string &proto_file_path) : enemy_detected_(false),
                                                            armor_detection_actionlib_client_("armor_detection_node_action", true)
  {

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") +
                           "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    // referee system
    referee_info["master"] = boost::make_shared<RefereeSystemInfo>();
    referee_info["wing"] = boost::make_shared<RefereeSystemInfo>();
    RefereeSubscribe("master");
    RefereeSubscribe("wing");

    if (!decision_config.simulate())
    {
      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }
  }

  void RefereeSubscribe(std::string robot_name)
  {
    // std::vector<std::string> referee_topic_names = {
    //     "game_status",
    //     "game_result",
    //     "game_survivor",
    //     "field_bonus_status",
    //     "field_supplier_status",
    //     "robot_status",
    //     "robot_heat",
    //     "robot_bonus",
    //     "robot_damage",
    //     "robot_shoot"};

    // for (auto topic_name : referee_topic_names)
    // {
    //   ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>("/" + robot_name + "/" + topic_name, 100, boost::bind(&Blackboard::RefereeCallback, this, _1, topic_name, robot_name));
    // }
    nh_.subscribe<roborts_msgs::GameStatus>("/" + robot_name + "/game_status", 100, boost::bind(&Blackboard::GameStatusCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::GameResult>("/" + robot_name + "/game_result", 100, boost::bind(&Blackboard::GameResultCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::GameSurvivor>("/" + robot_name + "/game_survivor", 100, boost::bind(&Blackboard::GameSurvivorCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::BonusStatus>("/" + robot_name + "/field_bonus_status", 100, boost::bind(&Blackboard::BonusStatusCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::SupplierStatus>("/" + robot_name + "/field_supplier_status", 100, boost::bind(&Blackboard::SupplierStatusCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::RobotHeat>("/" + robot_name + "/robot_heat", 100, boost::bind(&Blackboard::RobotHeatCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::RobotBonus>("/" + robot_name + "/robot_bonus", 100, boost::bind(&Blackboard::RobotBonusCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::RobotStatus>("/" + robot_name + "/robot_status", 100, boost::bind(&Blackboard::RobotStatusCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::RobotDamage>("/" + robot_name + "/robot_damage", 100, boost::bind(&Blackboard::RobotDamageCallback, this, _1, robot_name));

    nh_.subscribe<roborts_msgs::RobotShoot>("/" + robot_name + "/robot_shoot", 100, boost::bind(&Blackboard::RobotShootCallback, this, _1, robot_name));
  }

  ~Blackboard() = default;

  // referee

  void GameStatusCallback(const roborts_msgs::GameStatusConstPtr &data, std::string id)
  {
    referee_info[id]->game_status.reset(data);
  }

  void GameResultCallback(const roborts_msgs::GameResultConstPtr &data, std::string id)
  {
    referee_info[id]->game_result.reset(data);
  }
  void GameSurvivorCallback(const roborts_msgs::GameSurvivorConstPtr &data, std::string id)
  {
    referee_info[id]->game_survivor.reset(data);
  }
  void BonusStatusCallback(const roborts_msgs::BonusStatusConstPtr &data, std::string id)
  {
    referee_info[id]->bonus_status.reset(data);
  }
  void SupplierStatusCallback(const roborts_msgs::SupplierStatusConstPtr &data, std::string id)
  {
    referee_info[id]->supplier_status.reset(data);
  }
  void RobotHeatCallback(const roborts_msgs::RobotHeatConstPtr &data, std::string id)
  {
    referee_info[id]->robot_heat.reset(data);
  }
  void RobotBonusCallback(const roborts_msgs::RobotBonusConstPtr &data, std::string id)
  {
    referee_info[id]->robot_bonus.reset(data);
  }
  void RobotStatusCallback(const roborts_msgs::RobotStatusConstPtr &data, std::string id)
  {
    referee_info[id]->robot_status.reset(data);
  }
  void RobotDamageCallback(const roborts_msgs::RobotDamageConstPtr &data, std::string id)
  {
    referee_info[id]->robot_damage.reset(data);
  }
  void RobotShootCallback(const roborts_msgs::RobotShootConstPtr &data, std::string id)
  {
    referee_info[id]->robot_shoot.reset(data);
  }

  int GetHP(std::string robot_name)
  {
    return referee_info[robot_name]->robot_status->remain_hp;
  }

  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr &feedback)
  {
    if (feedback->detected)
    {
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
                                  camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if (GetDistance(global_pose_msg, enemy_pose_) > 0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2)
        {
          enemy_pose_ = global_pose_msg;
        }
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    }
    else
    {
      enemy_detected_ = false;
    }
  }

  geometry_msgs::PoseStamped GetEnemy() const
  {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const
  {
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
  {
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const
  {
    return goal_;
  }

  bool IsNewGoal()
  {
    if (new_goal_)
    {
      new_goal_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2)
  {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2)
  {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose()
  {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap()
  {
    return costmap_ptr_;
  }

  const CostMap2D *GetCostMap2D()
  {
    return costmap_2d_;
  }

  const unsigned char *GetCharMap()
  {
    return charmap_;
  }

private:
  ros::NodeHandle nh_;
  void UpdateRobotPose()
  {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try
    {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex)
    {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D *costmap_2d_;
  unsigned char *charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H

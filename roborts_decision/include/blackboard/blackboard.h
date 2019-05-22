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

#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/ShootCmd.h"

#include "io/io.h"
#include "../../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "utils/line_iterator.h"

namespace roborts_decision
{
//! structure to store msg from referee system
struct RefereeSystemInfo
{
  roborts_msgs::GameStatus game_status;
  roborts_msgs::GameResult game_result;
  roborts_msgs::GameSurvivor game_survivor;
  roborts_msgs::BonusStatus bonus_status;
  roborts_msgs::SupplierStatus supplier_status;
  roborts_msgs::RobotStatus robot_status;
  roborts_msgs::RobotHeat robot_heat;
  roborts_msgs::RobotBonus robot_bonus;
  roborts_msgs::RobotDamage robot_damage;
  roborts_msgs::RobotShoot robot_shoot;
  RefereeSystemInfo() : game_status(),
                        game_result(),
                        game_survivor(),
                        bonus_status(),
                        supplier_status(),
                        robot_status(),
                        robot_heat(),
                        robot_bonus(),
                        robot_damage(),
                        robot_shoot(){

                        };
};

class Blackboard
{
public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef std::shared_ptr<Blackboard const> ConstPtr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  // config
  roborts_decision::DecisionConfig decision_config;
  // dictionary, used to store referee info for each robot
  std::map<std::string /*robot_name*/, std::shared_ptr<RefereeSystemInfo>> referee_info;

  ChassisExecutor::Ptr chassis_executor;
  GimbalExecutor::Ptr gimbal_executor;

  // RefereeSystemInfo wing_status;
  // RefereeSystemInfo master_status;
  explicit Blackboard(const std::string &proto_file_path) : enemy_detected_(false),
                                                            armor_detection_actionlib_client_("armor_detection_node_action", true),
                                                            nh_priv_("~")
  {

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") +
                           "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);

    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    chassis_executor = std::make_shared<ChassisExecutor>();
    gimbal_executor = std::make_shared<GimbalExecutor>();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    referee_info = std::map<std::string /*robot_name*/, std::shared_ptr<RefereeSystemInfo>>();

    nh_.getParam("tf_prefix", robot_name_);
    ROS_INFO("Robot Name: %s", robot_name_.c_str());

    // init bullet count
    bullet_count_ = (robot_name_ == robot_0_) ? 40 : 0;
    fric_wheel_srv_ = nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    shoot_srv_ = nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    // is_master = decision_config.master();
    // referee system
    referee_info[robot_0_] = std::make_shared<RefereeSystemInfo>();
    referee_info[robot_1_] = std::make_shared<RefereeSystemInfo>();
    RefereeSubscribe(robot_0_);
    RefereeSubscribe(robot_1_);
    ROS_INFO("Referee interface Initialized ");
    // IsBlue();
    // ROS_INFO("Referee interface Initialized ");
    //TODO: wait for referee msgs

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

  void Supply()
  {
    roborts_msgs::ProjectileSupply sup_msg;
    sup_msg.number = 50;
    if (robot_name_ == "robot_0")
      robot_master_supply_pub_.publish(sup_msg);
    else
      robot_wing_supply_pub_.publish(sup_msg);
  }

  bool TurnOnFricWheel()
  {
    roborts_msgs::FricWhl srv;
    srv.request.open = true;
    int i = 10;
    while (i-- >= 0)
    {
      if (fric_wheel_srv_.call(srv))
      {
        ROS_INFO("Fric received: %d", (int)srv.response.received);
        if (srv.response.received)
          return true;
      }
      else
      {
        ROS_ERROR("Failed to turn on fric wheel");
        return false;
      }
    }
    ROS_ERROR("Failed to turn on fric wheel");
  }

  void TurnOffFricWheel()
  {
    roborts_msgs::FricWhl srv;
    srv.request.open = false;
    int i = 10;
    while (i-- >= 0)
      if (fric_wheel_srv_.call(srv))
      {
        ROS_INFO("Fric received: %d", (int)srv.response.received);
        if (srv.response.received)
          return;
      }
      else
      {
        ROS_ERROR("Failed to turn on fric wheel");
        return;
      }
  }

  bool Shoot(int num)
  {
    roborts_msgs::ShootCmd srv;
    srv.request.mode = 1; //0 1 2 stop once continuous
    srv.request.number = num;
    while (true)
    {
      if (shoot_srv_.call(srv))
      {
        ROS_INFO("Shoot received: %d", (int)srv.response.received);
        if (srv.response.received)
        {
          bullet_count_ -= num;
          return true;
        }
      }
      else
      {
        ROS_ERROR("Failed to call shoot service");
        return false;
      }
    }
  }

  bool IsNeedReload()
  {
    ROS_INFO("Bullet count: %d", bullet_count_);
    return (bullet_count_ <= 10);
  }

  bool IsReloadAvailable()
  {
    return (referee_info[robot_name_]->supplier_status.status == 1); //preparing
  }

  void RefereeSubscribe(std::string robot_name)
  {

    ROS_INFO("Initializing blackboard subscriber to referee msg");
    if (robot_name == "robot_0")
    {
      robot_master_supply_pub_ = nh_.advertise<roborts_msgs::ProjectileSupply>("/" + robot_name + "/projectile_supply", 1);

      game_status_master_sub_ = nh_.subscribe<roborts_msgs::GameStatus>("/" + robot_name + "/game_status", 100, boost::bind(&Blackboard::GameStatusCallback, this, _1, robot_name));

      game_result_master_sub_ = nh_.subscribe<roborts_msgs::GameResult>("/" + robot_name + "/game_result", 100, boost::bind(&Blackboard::GameResultCallback, this, _1, robot_name));

      game_survivor_master_sub_ = nh_.subscribe<roborts_msgs::GameSurvivor>("/" + robot_name + "/game_survivor", 100, boost::bind(&Blackboard::GameSurvivorCallback, this, _1, robot_name));

      bonus_status_master_sub_ = nh_.subscribe<roborts_msgs::BonusStatus>("/" + robot_name + "/field_bonus_status", 100, boost::bind(&Blackboard::BonusStatusCallback, this, _1, robot_name));

      supplier_status_master_sub_ = nh_.subscribe<roborts_msgs::SupplierStatus>("/" + robot_name + "/field_supplier_status", 100, boost::bind(&Blackboard::SupplierStatusCallback, this, _1, robot_name));

      robot_heat_master_sub_ = nh_.subscribe<roborts_msgs::RobotHeat>("/" + robot_name + "/robot_heat", 100, boost::bind(&Blackboard::RobotHeatCallback, this, _1, robot_name));

      robot_bonus_master_sub_ = nh_.subscribe<roborts_msgs::RobotBonus>("/" + robot_name + "/robot_bonus", 100, boost::bind(&Blackboard::RobotBonusCallback, this, _1, robot_name));

      robot_status_master_sub_ = nh_.subscribe<roborts_msgs::RobotStatus>("/" + robot_name + "/robot_status", 100, boost::bind(&Blackboard::RobotStatusCallback, this, _1, robot_name));

      robot_damage_master_sub_ = nh_.subscribe<roborts_msgs::RobotDamage>("/" + robot_name + "/robot_damage", 100, boost::bind(&Blackboard::RobotDamageCallback, this, _1, robot_name));

      robot_shoot_master_sub_ = nh_.subscribe<roborts_msgs::RobotShoot>("/" + robot_name + "/robot_shoot", 100, boost::bind(&Blackboard::RobotShootCallback, this, _1, robot_name));
    }
    else if (robot_name == "robot_1")
    {
      robot_wing_supply_pub_ = nh_.advertise<roborts_msgs::ProjectileSupply>("/" + robot_name + "/projectile_supply", 1);

      game_status_wing_sub_ = nh_.subscribe<roborts_msgs::GameStatus>("/" + robot_name + "/game_status", 100, boost::bind(&Blackboard::GameStatusCallback, this, _1, robot_name));

      game_result_wing_sub_ = nh_.subscribe<roborts_msgs::GameResult>("/" + robot_name + "/game_result", 100, boost::bind(&Blackboard::GameResultCallback, this, _1, robot_name));

      game_survivor_wing_sub_ = nh_.subscribe<roborts_msgs::GameSurvivor>("/" + robot_name + "/game_survivor", 100, boost::bind(&Blackboard::GameSurvivorCallback, this, _1, robot_name));

      bonus_status_wing_sub_ = nh_.subscribe<roborts_msgs::BonusStatus>("/" + robot_name + "/field_bonus_status", 100, boost::bind(&Blackboard::BonusStatusCallback, this, _1, robot_name));

      supplier_status_wing_sub_ = nh_.subscribe<roborts_msgs::SupplierStatus>("/" + robot_name + "/field_supplier_status", 100, boost::bind(&Blackboard::SupplierStatusCallback, this, _1, robot_name));

      robot_heat_wing_sub_ = nh_.subscribe<roborts_msgs::RobotHeat>("/" + robot_name + "/robot_heat", 100, boost::bind(&Blackboard::RobotHeatCallback, this, _1, robot_name));

      robot_bonus_wing_sub_ = nh_.subscribe<roborts_msgs::RobotBonus>("/" + robot_name + "/robot_bonus", 100, boost::bind(&Blackboard::RobotBonusCallback, this, _1, robot_name));

      robot_status_wing_sub_ = nh_.subscribe<roborts_msgs::RobotStatus>("/" + robot_name + "/robot_status", 100, boost::bind(&Blackboard::RobotStatusCallback, this, _1, robot_name));

      robot_damage_wing_sub_ = nh_.subscribe<roborts_msgs::RobotDamage>("/" + robot_name + "/robot_damage", 100, boost::bind(&Blackboard::RobotDamageCallback, this, _1, robot_name));

      robot_shoot_wing_sub_ = nh_.subscribe<roborts_msgs::RobotShoot>("/" + robot_name + "/robot_shoot", 100, boost::bind(&Blackboard::RobotShootCallback, this, _1, robot_name));
    }
    else
    {
      robot_wing_supply_pub_ = nh_.advertise<roborts_msgs::RobotStatus>("/projectile_supply", 1);
      game_status_wing_sub_ = nh_.subscribe<roborts_msgs::GameStatus>("/game_status", 100, boost::bind(&Blackboard::GameStatusCallback, this, _1, robot_name));

      game_result_wing_sub_ = nh_.subscribe<roborts_msgs::GameResult>("/game_result", 100, boost::bind(&Blackboard::GameResultCallback, this, _1, robot_name));

      game_survivor_wing_sub_ = nh_.subscribe<roborts_msgs::GameSurvivor>("/game_survivor", 100, boost::bind(&Blackboard::GameSurvivorCallback, this, _1, robot_name));

      bonus_status_wing_sub_ = nh_.subscribe<roborts_msgs::BonusStatus>("/field_bonus_status", 100, boost::bind(&Blackboard::BonusStatusCallback, this, _1, robot_name));

      supplier_status_wing_sub_ = nh_.subscribe<roborts_msgs::SupplierStatus>("/field_supplier_status", 100, boost::bind(&Blackboard::SupplierStatusCallback, this, _1, robot_name));

      robot_heat_wing_sub_ = nh_.subscribe<roborts_msgs::RobotHeat>("/robot_heat", 100, boost::bind(&Blackboard::RobotHeatCallback, this, _1, robot_name));

      robot_bonus_wing_sub_ = nh_.subscribe<roborts_msgs::RobotBonus>("/robot_bonus", 100, boost::bind(&Blackboard::RobotBonusCallback, this, _1, robot_name));

      robot_status_wing_sub_ = nh_.subscribe<roborts_msgs::RobotStatus>("/robot_status", 100, boost::bind(&Blackboard::RobotStatusCallback, this, _1, robot_name));

      robot_damage_wing_sub_ = nh_.subscribe<roborts_msgs::RobotDamage>("/robot_damage", 100, boost::bind(&Blackboard::RobotDamageCallback, this, _1, robot_name));

      robot_shoot_wing_sub_ = nh_.subscribe<roborts_msgs::RobotShoot>("/robot_shoot", 100, boost::bind(&Blackboard::RobotShootCallback, this, _1, robot_name));
    }
  }

  ~Blackboard() = default;

  // referee
  int GetHP(std::string robot_name)
  {
    return referee_info[robot_name]->robot_status.remain_hp;
  }

  bool IsMaster()
  {
    // return decision_config.master();
    return (robot_name_ == robot_0_);
  }

  bool IsWing()
  {
    // return !decision_config.master();
    return (robot_name_ == robot_0_);
  }

  std::string GetRobotName()
  {
    // if (IsMaster())
    //   return robot_0_;
    // else
    //   return robot_1_;
    return robot_name_;
  }

  std::string GetOtherRobotName()
  {
    if (!IsMaster())
      return robot_0_;
    else
      return robot_1_;
  }

  bool IsBlue()
  {
    return (robot_color_ == 1);
  }

  bool IsRed()
  {
    return (robot_color_ == 0);
  }

  void GameStatusCallback(const roborts_msgs::GameStatusConstPtr &data, const std::string id)
  {
    referee_info[id]->game_status = *data;

    if (referee_info[id]->game_status.game_status == 3)
      game_status_ = 3;
    else if (referee_info[id]->game_status.game_status == 4)
      game_status_ = 4;

    // ROS_INFO("rcev  game status %d", game_status_);
    // ROS_INFO("rcev  game status %d", referee_info[id]->game_status.game_status);
  }

  void GameResultCallback(const roborts_msgs::GameResultConstPtr &data, const std::string id)
  {
    referee_info[id]->game_result = *data;
  }
  void GameSurvivorCallback(const roborts_msgs::GameSurvivorConstPtr &data, const std::string id)
  {
    referee_info[id]->game_survivor = *data;
  }
  void BonusStatusCallback(const roborts_msgs::BonusStatusConstPtr &data, const std::string id)
  {
    referee_info[id]->bonus_status = *data;
  }
  void SupplierStatusCallback(const roborts_msgs::SupplierStatusConstPtr &data, const std::string id)
  {
    referee_info[id]->supplier_status = *data;
  }
  void RobotHeatCallback(const roborts_msgs::RobotHeatConstPtr &data, const std::string id)
  {
    referee_info[id]->robot_heat = *data;
  }
  void RobotBonusCallback(const roborts_msgs::RobotBonusConstPtr &data, const std::string id)
  {
    referee_info[id]->robot_bonus = *data;
  }
  void RobotStatusCallback(const roborts_msgs::RobotStatusConstPtr &data, const std::string id)
  {
    referee_info[id]->robot_status = *data;
    robot_color_ = (referee_info[id]->robot_status.id < 10) ? 0 : 1;
    ROS_INFO("robot color %d", robot_color_);
  }
  void RobotDamageCallback(const roborts_msgs::RobotDamageConstPtr &data, const std::string id)
  {
    referee_info[id]->robot_damage = *data;
  }
  void RobotShootCallback(const roborts_msgs::RobotShootConstPtr &data, const std::string id)
  {
    referee_info[id]->robot_shoot = *data;
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

  int GetSupplierStatus()
  {
    // uint8 CLOSE = 0
    // uint8 PREPARING = 1
    // uint8 SUPPLYING = 2
    // uint8 status
    return referee_info[robot_name_]->supplier_status.status;
  }

  int GetBonusStatus()
  {
    // uint8 UNOCCUPIED = 0
    // uint8 BEING_OCCUPIED= 1
    // uint8 OCCUPIED = 2
    if (IsBlue())
      return referee_info[robot_name_]->bonus_status.blue_bonus;
    else
      return referee_info[robot_name_]->bonus_status.red_bonus;
  }

  // True if bonus is activated for this robot
  bool IsBonusActivated()
  {
    return referee_info[robot_name_]->robot_bonus.bonus;
  }

  bool IsBonusUnoccupied()
  {
    if (GetBonusStatus() == 0)
      return true;
  }

  bool IsBonusOccupied()
  {
    if (GetBonusStatus() == 2)
      return true;
  }

  //! True if nonus not occupied and bonus not activated
  bool IsBonusAvailable()
  {
    ROS_INFO("bonus occupied %d, bonus activated %d", (int)IsBonusUnoccupied(), (int)IsBonusActivated());
    // return false;
    return IsBonusUnoccupied() && !IsBonusActivated();
  }

  bool IsEnemyDetected() const
  {
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // game status
  bool IsFiveSecondCD()
  {
    ROS_INFO("game status %d", game_status_);
    return (game_status_ == 3);
    // if (referee_info[robot_name_]->game_status.game_status == 3)
    //   return true;
    // else
    //   return false;
  }

  bool IsGameStart()
  {
    ROS_INFO("game status %d", game_status_);
    return (game_status_ == 4);
    // if (referee_info[robot_name_]->game_status.game_status == 4)
    //   return true;
    // else
    //   return false;
  }

  int GetRemainTime()
  {
    referee_info[robot_name_]->game_status.remaining_time;
  }

  //! Bullet
  void AddBullet(int count)
  {
    bullet_count_ += count;
    ROS_INFO("Bullet number: %d", bullet_count_);
  }

  void SetBullet(int count)
  {
    bullet_count_ = count;
    ROS_INFO("Bullet number: %d", bullet_count_);
  }

  int GetBulletCount()
  {
    return bullet_count_;
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

  geometry_msgs::PoseStamped GetReloadGoal()
  {
    reload_goal_.header.frame_id = "/map";
    reload_goal_.pose.orientation.x = 0;
    reload_goal_.pose.orientation.y = 0;
    reload_goal_.pose.orientation.z = 0;
    reload_goal_.pose.orientation.w = 1;

    reload_goal_.pose.position.x = 4;
    reload_goal_.pose.position.y = 0.5;
    reload_goal_.pose.position.z = 0;

    return reload_goal_;
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
    UpdateRobotPose(robot_name_);
    return robot_map_pose_;
  }

  const geometry_msgs::PoseStamped GetOtherRobotMapPose()
  {
    UpdateRobotPose(GetOtherRobotName());
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
  const std::string robot_0_ = "robot_0";
  const std::string robot_1_ = "robot_1";
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::Subscriber game_status_master_sub_;
  ros::Subscriber game_result_master_sub_;
  ros::Subscriber game_survivor_master_sub_;
  ros::Subscriber bonus_status_master_sub_;
  ros::Subscriber supplier_status_master_sub_;
  ros::Subscriber robot_status_master_sub_;
  ros::Subscriber robot_heat_master_sub_;
  ros::Subscriber robot_bonus_master_sub_;
  ros::Subscriber robot_damage_master_sub_;
  ros::Subscriber robot_shoot_master_sub_;
  ros::Subscriber game_status_wing_sub_;
  ros::Subscriber game_result_wing_sub_;
  ros::Subscriber game_survivor_wing_sub_;
  ros::Subscriber bonus_status_wing_sub_;
  ros::Subscriber supplier_status_wing_sub_;
  ros::Subscriber robot_status_wing_sub_;
  ros::Subscriber robot_heat_wing_sub_;
  ros::Subscriber robot_bonus_wing_sub_;
  ros::Subscriber robot_damage_wing_sub_;
  ros::Subscriber robot_shoot_wing_sub_;
  ros::Publisher robot_wing_supply_pub_;
  ros::Publisher robot_master_supply_pub_;
  bool is_blue;
  int game_status_;

  void UpdateRobotPose(std::string robot_name)
  {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "/" + robot_name + "/" + "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try
    {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("/map", robot_pose, robot_map_pose_);
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
  geometry_msgs::PoseStamped reload_goal_;
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

  //! bullet
  int bullet_count_;
  ros::ServiceClient fric_wheel_srv_;
  ros::ServiceClient shoot_srv_;

  std::string robot_name_;
  int robot_id_;

  int robot_color_; //0 red, 1 blue
};                  // namespace roborts_decision
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H

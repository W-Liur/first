/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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

#ifndef MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#define MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <ros/ros.h>

#include "messages/ArmorDetectionAction.h"
#include "messages/LocalizationAction.h"

#include "common/io.h"
#include "modules/decision/proto/decision.pb.h"


namespace rrts{
namespace decision {

class Blackboard {
 public:
  //lwh
  //! enum
  enum actionState{
      IDEL,
      RUNNING,
      SUCCESS
  };
  typedef std::shared_ptr<Blackboard> Ptr;
  Blackboard():
      localization_actionlib_client_("localization_node_action", true),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      actiontype_(0),
      enemy_found_(false),   //是否有敌人 if have enmy
      has_goal_(false),      //是否有目标点 if have goals
      defend_found_(true),
      nodepart_found_(true), //是否没有出发 if haven't depart
      occupy_found_(true),
      monitor_found_(true),
      goal_end_(3),      //是否是最后一个点 if is the last point
      monitor_(actionState::RUNNING),      // RUNNING:执行； SUCCESS:终止；
      defend_(actionState::SUCCESS),       // RUNNING:执行； SUCCESS:终止；
      occupy_(actionState::RUNNING),       // RUNNING:执行； SUCCESS:终止；
      patrol_(actionState::SUCCESS),       // RUNNING:执行； SUCCESS:终止； //是否进行巡逻     if patrol
      depart_(actionState::SUCCESS),          // IDEL：执行规划路径； RUNNING:执行单片机路径； SUCCESS:终止； //是否出发  if depart
      planDepart_(true)                    //是否使用规划出发  if depart using planning
  {
    LoadParam();
    ros::NodeHandle rviz_nh("move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &Blackboard::EnemyCallback, this);
    //lwh
    defend_sub_ = rviz_nh.subscribe("defend",2,&Blackboard::defendCallback,this);
    depart_sub_ = rviz_nh.subscribe("depart",2,&Blackboard::departCallback,this);
    occupy_sub_ = rviz_nh.subscribe("occupy",2,&Blackboard::occupyCallback,this);
    monitor_pub_ = rviz_nh.subscribe("monitor",2,&Blackboard::monitorCallback,this);
    planning_sub_ = rviz_nh.subscribe("planning",2,&Blackboard::planningCallback,this);
    // Connect to localization server and start
    localization_actionlib_client_.waitForServer();
    LOG_INFO << "Localization module has been connected!";
    localization_goal_.command = 1;
    localization_actionlib_client_.sendGoal(localization_goal_);

  };
  ~Blackboard() {};
//！回调函数
  void EnemyCallback(const geometry_msgs::PoseStamped::ConstPtr & enemy){
     SetEnemy1(*enemy);SetEnemy2(*enemy);
     switch (actiontype_){
         case 1 :
             defend_goals_iter_ = defend_goals_.begin();
             defend_found_ = true;
             LOG_ERROR<<"defend update!";
             break;
         case 2 :
             depart_goals_iter_ = depart_goals_.begin();
             nodepart_found_ = true;
             LOG_ERROR<<"depart update!";
             break;
         case 3 :
             occupy_goals_iter_ = occupy_goals_.begin();
             occupy_found_ = true;
             LOG_ERROR<<"occupy update!";
             break;
         case 4 :
             monitor_goals_iter_ = monitor_goals_.begin();
             monitor_found_ = true;
             LOG_ERROR<<"monitor update!";
             break;
         default:
             LOG_INFO<<"default!";
     }
  }
  void defendCallback(const nav_msgs::Path & poses){
  }
  void departCallback(const nav_msgs::Path & poses){
      depart_=actionState::IDEL;
  }
  void occupyCallback(const nav_msgs::Path & poses){
    occupy_=actionState::RUNNING;
    occupy_found_ = true;
  }
  void monitorCallback(const nav_msgs::Path & poses) {
  }
  void planningCallback(const nav_msgs::Path & poses){
      patrol_=actionState::RUNNING;
  }
//!是否获取目标点及是否给路径规划发送目标点点
  bool HasGoal(){
    return has_goal_;
  }
  void SetGoal1(const geometry_msgs::PoseStamped & goal) {
    goal1_ = goal;
    has_goal_=true;
  }
  void SetGoal2(const geometry_msgs::PoseStamped & goal) {
      goal2_ = goal;
      has_goal_=true;
  }
  geometry_msgs::PoseStamped GetGoal1() {
    has_goal_ = false;
    return goal1_;
  }
  geometry_msgs::PoseStamped GetGoal2() {
    has_goal_ = false;
    return goal2_;
  }
//!获取敌人目标点
  bool HasEnemy() {
    return enemy_found_;
  }
  void SetEnemy1(const geometry_msgs::PoseStamped &enemy) {
    enemy1_ = enemy;
    enemy_found_ = true;
  }
  void SetEnemy2(const geometry_msgs::PoseStamped &enemy) {
      enemy2_ = enemy;
      enemy_found_ = true;
  }
  geometry_msgs::PoseStamped GetEnemy1() {
    enemy_found_ = false;
    return enemy1_;
  }
  geometry_msgs::PoseStamped GetEnemy2() {
      enemy_found_ = false;
      return enemy2_;
  }
//!询问是否防御
  bool HasDefend() {
      if(defend_==actionState::RUNNING){
          return defend_found_;
      } else if(defend_==actionState::SUCCESS){
          return false;
      }
  }
//!获取防御目标点坐标
  geometry_msgs::PoseStamped GetDefend() {
      static int iter=1;
      actiontype_ = 1;
      defend_goals_iter_ = defend_goals_iter_ + iter;
      geometry_msgs::PoseStamped poseZero;
      if (defend_goals_iter_ == defend_goals_.end()) {
  //          defend_found_=false;
          iter = 0;
          return poseZero;
      }
      if (defend_goals_iter_ == defend_goals_.end()-1) {
          goal_end_=1;
      } else {
          goal_end_=3;
      }
      return *defend_goals_iter_;
  }
//!询问是否出发
  bool HasNoDepart() {
    if(depart_==actionState::IDEL) {
        planDepart_=true;
        return nodepart_found_;
    } else if(depart_==actionState::RUNNING){
        planDepart_=false;
        return true;
    } else if(depart_==actionState::SUCCESS){
        planDepart_=false;
        return false;
    }
  }
  bool GetPlanDepart() {
    return planDepart_;
  }
//!获取出发点坐标
  geometry_msgs::PoseStamped GetDepart1() {
      actiontype_ = 2;
      geometry_msgs::PoseStamped poseZero;
      depart_goals_iter_;
      if (depart_goals_iter_ == depart_goals_.end()) {
          nodepart_found_=false;
          return poseZero;
      }
      return *depart_goals_iter_;
  }
  geometry_msgs::PoseStamped GetDepart2() {
      actiontype_ = 2;
      geometry_msgs::PoseStamped poseZero;
      depart_goals_iter_++;
      if (depart_goals_iter_ == depart_goals_.end()) {
          nodepart_found_=false;
          return poseZero;
      }
      if (depart_goals_iter_ == depart_goals_.end()-1) {
          goal_end_=2;
      } else {
          goal_end_=3;
      }
      return *depart_goals_iter_;
  }
//!询问是否占buff点
  bool HasOccupy() {
      if(occupy_==actionState::RUNNING){
          return occupy_found_;
      } else if(occupy_==actionState::SUCCESS){
          return false;
      }
  }
//！获取buff点坐标
  geometry_msgs::PoseStamped GetOccupy() {
      static int iter=1;
      actiontype_ = 3;
      occupy_goals_iter_ = occupy_goals_iter_ + iter;
      geometry_msgs::PoseStamped poseZero;
      if (occupy_goals_iter_ == occupy_goals_.end()) {
          occupy_found_=false;
          iter = 0;
          return poseZero;
      }
      if (occupy_goals_iter_ == occupy_goals_.end()-1) {
          goal_end_=1;
      } else {
          goal_end_=3;
      }
      return *occupy_goals_iter_;
  }
//！询问是否单点监控
  bool HasMonitor() {
      if(monitor_==actionState::RUNNING){
          return monitor_found_;
      } else if(monitor_==actionState::SUCCESS){
          return false;
      }
  }
//!获取监控目标点坐标
  geometry_msgs::PoseStamped GetMonitor() {
      static int iter=1;
      actiontype_ = 4;
      monitor_goals_iter_ = monitor_goals_iter_ + iter;
      geometry_msgs::PoseStamped poseZero;
      if (monitor_goals_iter_ == monitor_goals_.end()) {
          monitor_found_=false;
          iter = 0;
          return poseZero;
      }
      if (monitor_goals_iter_ == monitor_goals_.end()-1) {
          goal_end_=1;
      } else {
          goal_end_=3;
      }
      return *monitor_goals_iter_;
  }
//!询问是否巡逻
  bool HasPartol() {
      if(patrol_==actionState::RUNNING){
          return true;
      } else if(patrol_==actionState::SUCCESS){
          return false;
      } else {
          return false;
      }
  }
//!获取巡逻点坐标
  geometry_msgs::PoseStamped GetPatrol1() {
    actiontype_ = 0;
    select_patrol_goals_iter_;
    goal_end_=3;
//    if (select_patrol_goals_iter_ == select_patrol_goals_.end()) {
//        select_patrol_goals_iter_ = select_patrol_goals_.begin();
//    }
    return *select_patrol_goals_iter_;
  }
  geometry_msgs::PoseStamped GetPatrol2() {
    actiontype_ = 0;
    select_patrol_goals_iter_++;
    goal_end_=3;
    if (select_patrol_goals_iter_ == select_patrol_goals_.end()) {
        select_patrol_goals_iter_ = select_patrol_goals_.begin();
    }
    return *select_patrol_goals_iter_;
  }
//！判断发送目标点的个数
  int HasGoalEnd() {
      return goal_end_;
  }

 private:
  void TranformPoint(std::vector<geometry_msgs::PoseStamped>& goals_,unsigned int size_,const rrts::decision::Points Points) {
      for (int i = 0; i != size_; i++) {
          goals_[i].header.frame_id = "map";
          goals_[i].pose.position.x = Points.point(i).x();
          goals_[i].pose.position.y = Points.point(i).y();
          goals_[i].pose.position.z = Points.point(i).z();

          tf::Quaternion quaternion = tf::createQuaternionFromRPY(Points.point(i).roll(),
                                                                  Points.point(i).pitch(),
                                                                  Points.point(i).yaw());
          goals_[i].pose.orientation.x = quaternion.x();
          goals_[i].pose.orientation.y = quaternion.y();
          goals_[i].pose.orientation.z = quaternion.z();
          goals_[i].pose.orientation.w = quaternion.w();
      }
  }
  void LoadParam() {
    rrts::decision::DecisionConfig decision_config;
    std::string file_name = "modules/decision/config/decision.prototxt";
    rrts::common::ReadProtoFromTextFile(file_name, &decision_config);

    //defend goal config
    unsigned int defend_point_size = decision_config.defendpoint().point().size();
    defend_goals_.resize(defend_point_size);
    TranformPoint(defend_goals_,defend_point_size,decision_config.defendpoint());
    defend_goals_iter_ = defend_goals_.begin();

    //depart goal config
    unsigned int depart_point_size = decision_config.departpoint().point().size();
    depart_goals_.resize(depart_point_size);
    TranformPoint(depart_goals_,depart_point_size,decision_config.departpoint());
    depart_goals_iter_ = depart_goals_.begin();

    //occupy goal config
    unsigned int occupy_point_size = decision_config.occupypoint().point().size();
    occupy_goals_.resize(occupy_point_size);
    TranformPoint(occupy_goals_,occupy_point_size,decision_config.occupypoint());
    occupy_goals_iter_ = occupy_goals_.begin();

    //monitor goal config
    unsigned int monitor_point_size = decision_config.monitorpoint().point().size();
    monitor_goals_.resize(monitor_point_size);
    TranformPoint(monitor_goals_,monitor_point_size,decision_config.monitorpoint());
    monitor_goals_iter_ = monitor_goals_.begin();

    //patrol goal config
    unsigned int patrol_point_size = decision_config.patrolpoint().point().size();
    patrol_goals_.resize(patrol_point_size);
    TranformPoint(patrol_goals_,patrol_point_size,decision_config.patrolpoint());
    patrol_goals_iter_ = patrol_goals_.begin();

    //onepatrol goal config
    unsigned int onepatrol_point_size = decision_config.onepatrolpoint().point().size();
    onepatrol_goals_.resize(onepatrol_point_size);
    TranformPoint(onepatrol_goals_,onepatrol_point_size,decision_config.onepatrolpoint());
    onepatrol_goals_iter_ = onepatrol_goals_.begin();

    //twopatrol goal config
    unsigned int twopatrol_point_size = decision_config.twopatrolpoint().point().size();
    twopatrol_goals_.resize(twopatrol_point_size);
    TranformPoint(twopatrol_goals_,twopatrol_point_size,decision_config.twopatrolpoint());
    twopatrol_goals_iter_ = twopatrol_goals_.begin();

    //threepatrol goal config
    unsigned int threepatrol_point_size = decision_config.threepatrolpoint().point().size();
    threepatrol_goals_.resize(threepatrol_point_size);
    TranformPoint(threepatrol_goals_,threepatrol_point_size,decision_config.threepatrolpoint());
    threepatrol_goals_iter_ = threepatrol_goals_.begin();

    //smallpatrol goal config
    unsigned int smallpatrol_point_size = decision_config.smallpatrolpoint().point().size();
    smallpatrol_goals_.resize(smallpatrol_point_size);
    TranformPoint(smallpatrol_goals_,smallpatrol_point_size,decision_config.smallpatrolpoint());
    smallpatrol_goals_iter_ = smallpatrol_goals_.begin();

    switch (decision_config.selectpatrolmode().mode()) {
      case 1 :
          select_patrol_goals_.assign(patrol_goals_.begin(),patrol_goals_.end());
          select_patrol_goals_iter_ = select_patrol_goals_.begin();
          LOG_ERROR<<"patrol!";
          break;
      case 2 :
          select_patrol_goals_.assign(onepatrol_goals_.begin(),onepatrol_goals_.end());
          select_patrol_goals_iter_ = select_patrol_goals_.begin();
          LOG_ERROR<<"onepatrol!";
          break;
      case 3 :
          select_patrol_goals_.assign(twopatrol_goals_.begin(),twopatrol_goals_.end());
          select_patrol_goals_iter_ = select_patrol_goals_.begin();
          LOG_ERROR<<"twopatrol!";
          break;
      case 4 :
          select_patrol_goals_.assign(threepatrol_goals_.begin(),threepatrol_goals_.end());
          select_patrol_goals_iter_ = select_patrol_goals_.begin();
          LOG_ERROR<<"threepatrol!";
          break;
      case 5 :
          select_patrol_goals_.assign(smallpatrol_goals_.begin(),smallpatrol_goals_.end());
          select_patrol_goals_iter_ = select_patrol_goals_.begin();
          LOG_ERROR<<"smallpatrol!";
          break;
      default:
          LOG_ERROR<<"default!";
    }
  }
/*>>>>>>>>>>>>>>>>>>李文浩修改>>>>>>>>>>>>>>>>>>>>>>>>>*/
  //!
  ros::Subscriber enemy_sub_;
  //lwh
  //! 订阅节点
  ros::Subscriber defend_sub_ ,depart_sub_ ,occupy_sub_,monitor_pub_,planning_sub_;
  //! Action client
  actionlib::SimpleActionClient<messages::LocalizationAction> localization_actionlib_client_;
  actionlib::SimpleActionClient<messages::ArmorDetectionAction> armor_detection_actionlib_client_;

  //! Action goal
  messages::LocalizationGoal localization_goal_;
  messages::ArmorDetectionGoal armor_detection_goal_;
/*<<<<<<<<<<<<<<<<<<李文浩修改<<<<<<<<<<<<<<<<<<<<<<<<<*/
  //! Goal
  geometry_msgs::PoseStamped goal1_;
  geometry_msgs::PoseStamped goal2_;
  bool has_goal_;

  //! EnemyFound
  geometry_msgs::PoseStamped enemy1_;
  geometry_msgs::PoseStamped enemy2_;
  bool enemy_found_;

  //! DefendFound lwh
  bool defend_found_;

  //! NoDepartFound lwh
  bool nodepart_found_;

  //! OccupyFound lwh
  bool occupy_found_;

  //! MonitorFound lwh
  bool monitor_found_;

  //! GoalEnd lwh  1:single point; 2:double points; 3:mutiple points
  int goal_end_;
/*>>>>>>>>>>>>>>>>>>李文浩修改>>>>>>>>>>>>>>>>>>>>>>>>>*/
  //! Defend
  actionState defend_;
  std::vector<geometry_msgs::PoseStamped> defend_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator defend_goals_iter_;
  //! Depart
  actionState depart_;
  bool planDepart_;
  std::vector<geometry_msgs::PoseStamped> depart_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator depart_goals_iter_;
  //! Occupy
  actionState occupy_;
  std::vector<geometry_msgs::PoseStamped> occupy_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator occupy_goals_iter_;
  //! Monitor
  actionState monitor_;
  std::vector<geometry_msgs::PoseStamped> monitor_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator monitor_goals_iter_;
  //! Eslect Patrol
  std::vector<geometry_msgs::PoseStamped> select_patrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator select_patrol_goals_iter_;
  //! Patrol
  actionState patrol_;
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator patrol_goals_iter_;
  //! Onepatrol
  actionState onepatrol_;
  std::vector<geometry_msgs::PoseStamped> onepatrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator onepatrol_goals_iter_;
  //! Twopatrol
  actionState twopatrol_;
  std::vector<geometry_msgs::PoseStamped> twopatrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator twopatrol_goals_iter_;
  //! Threepatrol
  actionState threepatrol_;
  std::vector<geometry_msgs::PoseStamped> threepatrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator threepatrol_goals_iter_;
  //! Smallpatrol
  actionState smallpatrol_;
  std::vector<geometry_msgs::PoseStamped> smallpatrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator smallpatrol_goals_iter_;

  //! 执行类型
  int actiontype_;
};
} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H

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

#ifndef MODULE_DECISION_BEHAVIOR_TREE_CONDITION_BEHAVIOR_H
#define MODULE_DECISION_BEHAVIOR_TREE_CONDITION_BEHAVIOR_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "messages/GlobalPlannerAction.h"

#include "common/error_code.h"
#include "common/log.h"

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/behavior_tree/blackboard.h"

namespace rrts{
namespace decision {
    //lwh
    bool ifWaite = false;
    void sleepTime(int time) {
        static std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::seconds sleep_time =
                std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        LOG_ERROR<<"sleep time: "<<sleep_time.count();
        if (sleep_time > std::chrono::seconds(time)) {
            ifWaite = false;
        }
    }
class EnemyBehavior: public PreconditionNode{
 public:
  EnemyBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
      PreconditionNode("enemy_behavior",blackboard_ptr, child_node_ptr),track_location_(false){

  }
  virtual ~EnemyBehavior() = default;
  virtual void OnInitialize(){
    track_location_ = false;
    LOG_ERROR<<"EnemyBahavior OnInitialize!";
  }
/*<<<<<<<<<<<<<<<<<<李文浩修改<<<<<<<<<<<<<<<<<<<<<<<<<*/
  virtual bool Precondition(){
    LOG_ERROR<<"EnemyBehavior Precondition!";
    if(blackboard_ptr_->HasEnemy()){
      enemy1_ = blackboard_ptr_->GetEnemy1();
      blackboard_ptr_->SetGoal1(enemy1_);
      enemy2_ = blackboard_ptr_->GetEnemy2();
      blackboard_ptr_->SetGoal2(enemy2_);
      track_location_=true;
      return true;
    }
    else if(track_location_){
      return true;
    }
    return false;
  }
 private:
  geometry_msgs::PoseStamped enemy1_;
  geometry_msgs::PoseStamped enemy2_;
  bool track_location_;
};
/*>>>>>>>>>>>>>>>>>>李文浩修改>>>>>>>>>>>>>>>>>>>>>>>>>*/

//lwh defend
class DefendBehavior: public PreconditionNode{
public:
    DefendBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
            PreconditionNode("patrol_behavior",blackboard_ptr, child_node_ptr){

    }
    virtual ~DefendBehavior() = default;
    virtual void OnInitialize(){
        LOG_ERROR<<"DefendBehavior OnInitialize!";
        LOG_ERROR<<blackboard_ptr_->HasEnemy();
        LOG_ERROR<<blackboard_ptr_->HasNoDepart();
        if(!blackboard_ptr_->HasEnemy()&&blackboard_ptr_->HasDefend()){
            defend_goal_ = blackboard_ptr_->GetDefend();
            blackboard_ptr_->SetGoal1(defend_goal_);
        }
    }
    virtual bool Precondition(){
        LOG_ERROR<<"DefendBehavior Precondition!";
        if(!blackboard_ptr_->HasEnemy()&&blackboard_ptr_->HasDefend()){
            LOG_ERROR<<"DefendBehavior Precondition!";
            return true;
        }
        LOG_ERROR<<"DefendBehavior Precondition!";
        return false;
    }

private:
    geometry_msgs::PoseStamped defend_goal_;
};

//lwh
class DepartBehavior: public PreconditionNode{
public:
    DepartBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
            PreconditionNode("patrol_behavior",blackboard_ptr, child_node_ptr){

    }
    virtual ~DepartBehavior() = default;
    virtual void OnInitialize(){
        LOG_ERROR<<"DepartBehavior OnInitialize!";
        LOG_ERROR<<blackboard_ptr_->HasEnemy();
        LOG_ERROR<<blackboard_ptr_->HasNoDepart();
        if(!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&blackboard_ptr_->HasNoDepart()){
            if(blackboard_ptr_->GetPlanDepart()) {
                depart_goal1_ = blackboard_ptr_->GetDepart1();
                blackboard_ptr_->SetGoal1(depart_goal1_);
                depart_goal2_ = blackboard_ptr_->GetDepart2();
                blackboard_ptr_->SetGoal2(depart_goal2_);
            }
        }
    }
    virtual bool Precondition(){
        LOG_ERROR<<"DepartBehavior Precondition!";
        if(!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&blackboard_ptr_->HasNoDepart()){
            return true;
        }
        return false;
    }

private:
    geometry_msgs::PoseStamped depart_goal1_;
    geometry_msgs::PoseStamped depart_goal2_;
};

//lwh
class OccupyBehavior: public PreconditionNode{
public:
    OccupyBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
            PreconditionNode("patrol_behavior",blackboard_ptr, child_node_ptr){

    }
    virtual ~OccupyBehavior() = default;
    virtual void OnInitialize(){
        LOG_ERROR<<"OccupyBehavior OnInitialize!";
        LOG_ERROR<<blackboard_ptr_->HasEnemy();
        LOG_ERROR<<blackboard_ptr_->HasNoDepart();
        LOG_ERROR<<blackboard_ptr_->HasOccupy();
        if(!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&!blackboard_ptr_->HasNoDepart()&&(blackboard_ptr_->HasOccupy() || ifWaite)){
            occupy_goal_ = blackboard_ptr_->GetOccupy();
            if(blackboard_ptr_->HasOccupy()) {
                blackboard_ptr_->SetGoal1(occupy_goal_);
            } else {
                ifWaite =true;
                sleepTime(60);	//???
            }

        }
    }
    virtual bool Precondition(){
        LOG_ERROR<<"OccupyBehavior Precondition!";
        if(!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&!blackboard_ptr_->HasNoDepart()&&(blackboard_ptr_->HasOccupy() || ifWaite)){
            return true;
        }
        return false;
    }

private:
    geometry_msgs::PoseStamped occupy_goal_;
};

//lwh
class MonitorBehavior: public PreconditionNode{
public:
    MonitorBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
            PreconditionNode("patrol_behavior",blackboard_ptr, child_node_ptr){

    }
    virtual ~MonitorBehavior() = default;
/*<<<<<<<<<<<<<<<<<<李文浩修改<<<<<<<<<<<<<<<<<<<<<<<<<*/
    virtual void OnInitialize(){
        LOG_ERROR<<"PatrolBehavior OnInitialize!";
        LOG_ERROR<<blackboard_ptr_->HasPartol();LOG_ERROR<<blackboard_ptr_->HasEnemy();LOG_ERROR<<blackboard_ptr_->HasNoDepart();LOG_ERROR<<blackboard_ptr_->HasOccupy();
        if(!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&!blackboard_ptr_->HasNoDepart()&&!(blackboard_ptr_->HasOccupy() || ifWaite)&&blackboard_ptr_->HasMonitor()){
            monitor_goal_ = blackboard_ptr_->GetMonitor();
            blackboard_ptr_->SetGoal1(monitor_goal_);
        }
    }
    virtual bool Precondition(){
        LOG_ERROR<<"PatrolBehavior Precondition!";
        if(!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&!blackboard_ptr_->HasNoDepart()&&!(blackboard_ptr_->HasOccupy() || ifWaite)&&blackboard_ptr_->HasMonitor()){
            return true;
        }
        return false;
    }

private:
    geometry_msgs::PoseStamped monitor_goal_;
};

class PatrolBehavior: public PreconditionNode{
 public:
  PatrolBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
      PreconditionNode("patrol_behavior",blackboard_ptr, child_node_ptr){

  }
  virtual ~PatrolBehavior() = default;
/*<<<<<<<<<<<<<<<<<<李文浩修改<<<<<<<<<<<<<<<<<<<<<<<<<*/
  virtual void OnInitialize(){
    LOG_ERROR<<"PatrolBehavior OnInitialize!";
    LOG_ERROR<<blackboard_ptr_->HasPartol();LOG_ERROR<<blackboard_ptr_->HasEnemy();LOG_ERROR<<blackboard_ptr_->HasNoDepart();LOG_ERROR<<blackboard_ptr_->HasOccupy();
    if(blackboard_ptr_->HasPartol()&&!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&!blackboard_ptr_->HasNoDepart()&&!(blackboard_ptr_->HasOccupy() || ifWaite)&&!blackboard_ptr_->HasMonitor()){
      patrol_goal1_ = blackboard_ptr_->GetPatrol1();
      blackboard_ptr_->SetGoal1(patrol_goal1_);
      patrol_goal2_ = blackboard_ptr_->GetPatrol2();
      blackboard_ptr_->SetGoal2(patrol_goal2_);
    }
  }
  virtual bool Precondition(){
    LOG_ERROR<<"PatrolBehavior Precondition!";
    if(blackboard_ptr_->HasPartol()&&!blackboard_ptr_->HasEnemy()&&!blackboard_ptr_->HasDefend()&&!blackboard_ptr_->HasNoDepart()&&!(blackboard_ptr_->HasOccupy() || ifWaite)&&!blackboard_ptr_->HasMonitor()){
      return true;
    }
    return false;
  }

 private:
  geometry_msgs::PoseStamped patrol_goal1_;
  geometry_msgs::PoseStamped patrol_goal2_;
};
/*>>>>>>>>>>>>>>>>>>李文浩修改>>>>>>>>>>>>>>>>>>>>>>>>>*/
} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_CONDITION_BEHAVIOR_H

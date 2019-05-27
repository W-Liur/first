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


#ifndef MODULE_PERCEPTION_LOCALIZATION_LOCALIZATION_NODE_H
#define MODULE_PERCEPTION_LOCALIZATION_LOCALIZATION_NODE_H

#include <thread>

#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

//wyy
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/LaserScan.h>

#include "actionlib/server/simple_action_server.h"
#include "messages/LocalizationAction.h"

#include "modules/perception/localization/proto/localization_config.pb.h"
#include "modules/perception/localization/amcl/amcl.h"

#include "common/algorithm_factory.h"
#include "common/io.h"
#include "common/log.h"
#include "common/node_state.h"
#include "common/error_code.h"
#include "common/main_interface.h"
#include "common/rrts.h"

namespace rrts {
namespace perception {
namespace localization {

using rrts::common::NodeState;
using rrts::common::ErrorInfo;

/**
 * @brief Localization module
 */
class LocalizationNode : public rrts::common::RRTS {
 public:

    LocalizationNode(std::string name);

    /**
     * @brief Localization initialization
     * @return Returns errorcode
     */
    ErrorInfo Init() ;

  ~LocalizationNode();

  void ActionCallback(const messages::LocalizationGoal::ConstPtr &data);

 private:

  void StartMsgCallback();

  void StopMsgCallback();

  /**
   * @brief Static map msg callback function
   * @param map_msg Static map message
   */
  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);

  /**
   * @brief Initial pose estimation callback function
   * @param init_pose_msg Initial pose estimation msg
   */
  void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg);

  /**
   * @brief Laser scan msg callback funtion, now update amcl here.
   * @param laser_scan_msg Laser scan msg
   */
  void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg);

  //TODO(kevin.li) merge these functions to common::tf::utility
  double GetYawFromTfPose(const tf::Pose& tf_pose);

  bool GetOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                   math::Vec3d& pose,
                   const ros::Time& timestamp,
                   const std::string& frame_id);

  void TransformInitialPose(tf::Pose &pose_new, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  bool TransformLaserpose(const sensor_msgs::LaserScan &laser_scan_msg,
                          math::Vec3d &laser_pose);

  void TransformLaserscanToBaseFrame(double &angle_min, double &angle_increment,
                                     const sensor_msgs::LaserScan &laser_scan_msg);

  void TransfromHypPoseToMsg(const HypPose &hyp_pose, geometry_msgs::PoseWithCovarianceStamped &hyp_pose_msg);

  //wyy
  void TransformEnemyPoseToMsg(const math::Vec4d &enemy_pose, geometry_msgs::Pose &enemy_pose_msg);

  bool PublishUpdatedTF();

  void PublishLatestValidTF();

 private:

  std::thread localization_thread_;

  //Localization algorithm object
  std::unique_ptr<rrts::perception::localization::Amcl> amcl_ptr_ = nullptr;

  //TF
  tf::TransformBroadcaster* tf_broadcaster_ptr_ = nullptr;
  tf::TransformListener* tf_listener_ptr_ = nullptr;

  //ROS Subscriber
  //wyy
  ros::Subscriber dis_angle_sub_;

  ros::Subscriber initial_pose_sub_;
  ros::Subscriber map_sub_;
  
  //ros::Subscriber laser_scan_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_  = nullptr;
  
  tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_  = nullptr;

  //ROS Nodehandler
  ros::NodeHandle nh_;

  //ROS Publisher
  ros::Publisher pose_pub_;
  ros::Publisher particlecloud_pub_;

  //wyy ROS Publisher
  ros::Publisher near0_enemy_pub_;
  ros::Publisher near1_enemy_pub_;

  //Config param
  LocalizationConfig localization_config_;
  bool first_map_only_ = true;
  ros::Duration transform_tolerance_;

  //Status control
  actionlib::SimpleActionServer<messages::LocalizationAction> as_;
  NodeState node_state_;
  ErrorInfo error_info_;
  bool running_ = false;
  bool initialized_ = false;
  bool first_map_received_ = false;
  bool sent_first_transform_ = false;
  bool laser_init_ = false;
  bool laser_update_flag_ = true;
  bool latest_tf_valid_ = false;

  //Name Param
  std::string base_frame_;
  std::string global_frame_;
  std::string odom_frame_;

  //Data
  tf::Stamped<tf::Pose> latest_odom_pose_;
  tf::Stamped<tf::Pose> odom_to_map_;
  geometry_msgs::PoseArray particle_cloud_poses_;
  HypPose hyp_pose_;
  tf::Transform latest_tf_;
  ros::Time laser_msg_time_stamp_;

};
}
}
}


#endif //MODULE_PERCEPTION_LOCALIZATION_LOCALIZATION_NODE_H

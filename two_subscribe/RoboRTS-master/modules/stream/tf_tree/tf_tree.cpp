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

#include "modules/stream/tf_tree/tf_tree.h"
#include "common/main_interface.h"

namespace rrts {
namespace stream {
namespace tf_tree {

TFTree::TFTree(std::string name): rrts::common::RRTS::RRTS(name) {
  //wyy
    //wyy
  dis_angle_sub_=nh_.subscribe("dis_angle",2,&TFTree::DisAngleCallback,this);
  imu_data_sub_=nh_.subscribe("imu/data",10,&TFTree::ImuDataCallback,this);

  tf::Quaternion rotation(0,0,0,0);
  tf::Vector3 transformation(0.2,0,0);
  tf::Transform transform(rotation, transformation);
  dis_angle_= new tf_node(transform,"base_link","base_laser_link");
  trans_foot_link_ = new tf_node(transform, "base_footprint", "base_link");
  
  running_ = true;

  // running_ = false;
  // std::string file_name = "modules/stream/tf_tree/config/tf_tree.prototxt";
  // coordinate_trans_ = new CoordinateTrans;
  // bool read_status = rrts::common::ReadProtoFromTextFile<CoordinateTrans>(file_name, coordinate_trans_);
  // CHECK(read_status) << "Cannot open " << file_name;

  // tf_num_ = coordinate_trans_->transformation().size();
  // Execute();
}

//wyy
void TFTree::DisAngleCallback(const std_msgs::Float32::ConstPtr& dis_angle_msg){
  tf::Quaternion rotation=tf::createQuaternionFromYaw((*dis_angle_msg).data*3.1415926/180);
  tf::Vector3 transformation(-0.03,0,0);
  tf::Transform transform(rotation, transformation);
  dis_angle_->transform_=transform;
  tf::StampedTransform stamped_transform(dis_angle_->transform_,
                                           ros::Time::now(),
                                           "base_link",
                                           "base_laser_link");
  tf_broadcaster_.sendTransform(stamped_transform);
}
void TFTree::ImuDataCallback(const sensor_msgs::Imu::ConstPtr& imu_data_msg){
  tf::Quaternion rotation=tf::createQuaternionFromYaw((*imu_data_msg).orientation.x*3.1415926/180);
  tf::Vector3 transformation(0,0,0);
  tf::Transform transform(rotation, transformation);
  // trans_foot_link_->transform_=transform;
  tf::StampedTransform stamped_transform(transform,
                                           ros::Time::now(),
                                           "base_footprint",
                                           "base_link");
  tf_broadcaster_.sendTransform(stamped_transform);
}


void TFTree::Execute() {
  AddLeaf<CoordinateTrans>(coordinate_trans_);
  running_ = true;
  tf_tree_thread_ = new std::thread(&TFTree::PubStaticTF, this, tf_num_);
}

template<typename T>
void TFTree::AddLeaf(const T *coordinate_trans) {
  for (unsigned int i = 0; i < tf_num_; i++) {
    tf::Quaternion rotation(coordinate_trans->transformation(i).rotation().yaw(),
                            coordinate_trans->transformation(i).rotation().pitch(),
                            coordinate_trans->transformation(i).rotation().roll());

    tf::Vector3 transformation(coordinate_trans->transformation(i).translation().x(),
                               coordinate_trans->transformation(i).translation().y(),
                               coordinate_trans->transformation(i).translation().z());

    tf::Transform transform(rotation, transformation);
    tf_node node(transform,
                 coordinate_trans->transformation(i).base(),
                 coordinate_trans->transformation(i).sensor());
    tf_nodes_.push_back(node);
  }
}

void TFTree::PubStaticTF(unsigned int tf_num) {
  ros::Rate rate(20);
  while (running_) {
    for (unsigned int i = 0; i < tf_num; i++) {
      tf::StampedTransform stamped_transform(tf_nodes_[i].transform_,
                                             ros::Time::now(),
                                             tf_nodes_[i].base_,
                                             tf_nodes_[i].sensor_);
      tf_broadcaster_.sendTransform(stamped_transform);
      rate.sleep();
    }
  }
}

TFTree::~TFTree() {
  running_ = false;
  if(tf_tree_thread_->joinable())
    tf_tree_thread_->join();
  delete tf_tree_thread_;
  delete coordinate_trans_;
}

} //namespace configure
} //namespace stream
} //namespace rrts

MAIN(rrts::stream::tf_tree::TFTree, "tf_tree");
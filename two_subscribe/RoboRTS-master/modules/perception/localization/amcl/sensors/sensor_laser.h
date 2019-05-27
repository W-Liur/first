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

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_SENSORS_SENSOR_LASER_H
#define MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_SENSORS_SENSOR_LASER_H

#include "modules/perception/localization/amcl/sensors/sensor_base.h"
#include "modules/perception/localization/amcl/math/math.h"
#include "modules/perception/localization/amcl/proto/amcl.pb.h"

namespace rrts {
namespace perception {
namespace localization {

/**
 * @brief Laser data class
 */
class SensorLaserData : public SensorData {
 public:
  /**
   * @brief Default constructor.
   */
  SensorLaserData() {
    ranges_mat.setZero();
  }

  /**
   * @brief Default destructor.
   */
  virtual ~SensorLaserData() {
    ranges_mat.resize(0, 2);
  }
 public:

  /**
   * @brief Ranges count of laser
   */
  int range_count;

  /**
   * @brief Max range of laser
   */
  double range_max;

  /**
   * @brief Laser ranges data
   */
  Eigen::MatrixX2d ranges_mat;
};

/**
 * @brief Laser sensor model class
 */
class SensorLaser : public SensorBase {
 public:
  /**
   * @brief Constructor function
   * @param max_beams Laser max beams number
   * @param map_ptr AmclMap object pointer
   */
  SensorLaser(size_t max_beams, const std::shared_ptr<AmclMap> &map_ptr);

  /**
   * @brief Initialize laser likelihood field model.
   * @param z_hit Measurement noise coefficient
   * @param z_rand Random measurement noise coefficient
   * @param sigma_hit Stddev of Gaussian model for laser hits
   * @param max_occ_dist Max distance at which we care about obstacles
   * @param do_beamskip Beam skipping option
   * @param beam_skip_distance Beam skipping distance
   * @param beam_skip_threshold Beam skipping threshold
   * @param beam_skip_error_threshold Threshold for the ratio of invalid beams -
   *                                  at which all beams are integrated to the likelihoods.
   *                                  This would be an error condition
   */
  void SetModelLikelihoodFieldProb(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist,
                                   bool do_beamskip,
                                   double beam_skip_distance,
                                   double beam_skip_threshold,
                                   double beam_skip_error_threshold);

  /**
   * @brief Update the filter based on the sensor model.
   * @param pf_ptr Particle filter object pointer
   * @param sensor_data_ptr Sensor data object pointer
   * @return Returns true if the filter has been updated.
   */
  virtual bool UpdateSensor(ParticleFilterPtr pf_ptr, SensorData *sensor_data_ptr);

  /**
   * @brief Determine the probability for the given pose. A more probabilistically correct model
   *        - also with the option to do beam skipping
   * @param sensor_laser_data_ptr Sensor data object pointer
   * @param sample_set_ptr Paticle sample set object pointer
   * @return Returns weight of given pose.
   */
  double LikelihoodFieldModelProb(SensorLaserData *sensor_laser_data_ptr,
                                  SampleSetPtr sample_set_ptr);

  /**
   * @brief Set laser pose
   * @param laser_pose Laser pose to set
   */
  void SetLaserPose(const math::Vec3d &laser_pose);

 private:

  void ResetTempData(int new_max_samples, int new_max_obs);

 private:

  LaserModel model_type_;

  double time_;

  /**
   * @brief AmclMap object pointer
   */
  std::shared_ptr<AmclMap> map_ptr_;

  math::Vec3d laser_pose_;
  int max_beams_;
  bool do_beamskip_;
  double beam_skip_distance_;
  double beam_skip_threshold_;
  double beam_skip_error_threshold_;

  int max_samples_ = 0;
  int max_obs_ = 0;
  std::vector<std::vector<double>> temp_obs_;

  // Laser model params
  // Mixture params for the components of the model; must sum to 1
  double z_hit_;
  double z_short_;
  double z_max_;
  double z_rand_;

  /**
   * @brief Stddev of Gaussian model for laser hits.
   */
  double sigma_hit_;
  
  //wyy
  public:
  std::vector<math::Vec4d> max_potential_enemy_;  //x,y,obs_range,angle
  math::Vec4d near0_enemy_, near1_enemy_;

};

}
}
}

#endif //MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_SENSORS_SENSOR_LASER_H

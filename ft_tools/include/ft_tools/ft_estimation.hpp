// Copyright 2023, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FT_TOOLS__FT_ESTIMATION_HPP_
#define FT_TOOLS__FT_ESTIMATION_HPP_
#include "ft_tools/ft_parameters.hpp"
// misc.
#include <Eigen/Core>
#include <string>
#include <memory>


namespace ft_tools
{

class FtEstimation
{
public:
  FtEstimation();
  ~FtEstimation();

  /**
   * @brief Initialize the wrench estimation process
   *
   * @param ft_calib_parameters F/T sensor calibration parameters
   * @param deadband Wrench deadband in [N,N,N,Nm,Nm,Nm]
   * @param interaction_frame_wrt_sensor_frame Homogeneous transformation \f${}^sT_i$\f
   * @param gravity_in_robot_base_frame Signed, i.e., typically [0, 0, -9.81]
   * @return true All OK
   * @return false Failed to initialize
   */
  bool init(
    const FtParameters & ft_calib_parameters,
    const Eigen::Matrix<double, 6, 1> & deadband,
    const Eigen::Isometry3d & interaction_frame_wrt_sensor_frame,
    const Eigen::Vector3d & gravity_in_robot_base_frame
  );

  /**
   * @brief Process raw wrench data and update the estimated and interaction wrenches.
   *
   * @param sensor_orientation Rotation matrix \f${}^rR_s$\f of the sensor w.r.t. the ref. frame
   * @param measured_wrench Raw wrench measurement expressed \f$\mathcal{F}_s$\f
   * @return true All OK
   * @return false Failed to process the wrench. As a result, estimated wrenches are invalid.
   */
  bool process_raw_wrench(
    const Eigen::Matrix<double, 3, 3> & sensor_orientation,
    const Eigen::Matrix<double, 6, 1> & measured_wrench
  );

  /// Get the estimated wrench in the sensor frame \f$\mathcal{F}_s$\f.
  const Eigen::Matrix<double, 6, 1> & get_estimated_wrench();

  /// Get the estimated interaction wrench in the reference frame \f$\mathcal{F}_r$\f.
  const Eigen::Matrix<double, 6, 1> & get_estimated_interaction_wrench();

  /**
   * @brief Set the estimation process parameters.
   *
   * @param ft_calib_parameters F/T sensor calibration parameters
   * @param deadband Wrench deadband in [N,N,N,Nm,Nm,Nm]
   * @param interaction_frame_wrt_sensor_frame Homogeneous transformation \f${}^sT_i$\f
   * @param gravity_in_robot_base_frame Signed, i.e., typically [0, 0, -9.81]
   * @return true All OK
   * @return false Failed to set the parameters (probably due to invalid params)
   */
  bool set_parameters(
    const FtParameters & ft_calib_parameters,
    const Eigen::Matrix<double, 6, 1> & deadband,
    const Eigen::Isometry3d & interaction_frame_wrt_sensor_frame,
    const Eigen::Vector3d & gravity_in_robot_base_frame
  );

  /**
   * @brief Set the interaction frame to sensor frame transformation \f${}^sT_i$\f
   *
   * @param interaction_frame_wrt_sensor_frame Homogeneous transformation \f${}^sT_i$\f
   * @return true All OK
   * @return false Failed to set the parameter
   */
  bool set_interaction_frame_wrt_sensor_frame(
    const Eigen::Isometry3d & interaction_frame_wrt_sensor_frame
  );

  /// Returns the current calibration parameters
  const FtParameters & get_ft_calibration();
  /// Returns the current deadband
  const Eigen::Matrix<double, 6, 1> & get_deadband();
  /// Returns the current transformation \f${}^sT_i$\f
  const Eigen::Isometry3d & get_transformation_interaction_to_sensor();

protected:
  bool is_initialized_ = false;
// Data
  Eigen::Matrix<double, 6, 1> raw_wrench_in_sensor_frame_ =
    Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> estimated_wrench_in_sensor_frame_ =
    Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> estimated_wrench_in_interaction_frame_ =
    Eigen::Matrix<double, 6, 1>::Zero();
  /// Rotation matrix \f${}^r T_s \f$
  Eigen::Matrix<double, 3, 3> sensor_orientation_wrt_ref_frame_;

// Parameters
  /// mass, com, f_0 and tau_0
  FtParameters ft_calib_parameters_;
  /// Gravity in robot base (typically, [0, 0, -9.81] m.s-2)
  Eigen::Vector3d gravity_in_robot_base_frame_ = Eigen::Vector3d::Zero();
  /**
   * Transformation \f${}^{\text{sensor}}T_{\text{interaction}}\f$ from interaction frame
   * (i.e. end-effector) to sensor
   */
  Eigen::Isometry3d interaction_frame_wrt_sensor_frame_;
  Eigen::Matrix<double, 6, 1> deadband_ = Eigen::Matrix<double, 6, 1>::Zero();

// Internal methods
  /// Remove weight (forces and torques)
  Eigen::Matrix<double, 6, 1> remove_weight(
    const Eigen::Matrix<double, 6,
    1> & raw_wrench_in_sensor_frame);
  /// Apply wrench deadband
  void apply_wrench_deadband(Eigen::Matrix<double, 6, 1> & wrench);
  /// Estimate wrench at interaction point
  Eigen::Matrix<double, 6, 1> estimate_interaction(
    const Eigen::Matrix<double, 6, 1> & wrench_in_sensor_frame
  );
};

}  // namespace ft_tools

#endif  // FT_TOOLS__FT_ESTIMATION_HPP_

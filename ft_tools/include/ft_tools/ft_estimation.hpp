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

  bool init(
    const FtParameters & ft_calib_parameters,
    const Eigen::Matrix<double, 6, 1> & deadband,
    const Eigen::Isometry3d & interaction_frame_to_sensor_frame
  );

  bool process_raw_wrench(
    const Eigen::Matrix<double, 3, 3> & sensor_orientation,
    const Eigen::Matrix<double, 6, 1> & measured_wrench
  );

  const Eigen::Matrix<double, 6, 1> & get_estimated_wrench();

  const Eigen::Matrix<double, 6, 1> & get_estimated_interaction_wrench();

  bool set_parameters(
    const FtParameters & ft_calib_parameters,
    const Eigen::Matrix<double, 6, 1> & deadband,
    const Eigen::Isometry3d & interaction_frame_to_sensor_frame
  );

  bool set_interaction_frame_to_sensor_frame(
    const Eigen::Isometry3d & interaction_frame_to_sensor_frame
  );

  const FtParameters & get_ft_calibration();
  const Eigen::Matrix<double, 6, 1> & get_deadband();
  const Eigen::Isometry3d & get_transformation_interaction_to_sensor();

protected:
  bool is_initialized_ = false;
// Data
  Eigen::Matrix<double, 6, 1> raw_wrench_in_sensor_frame_ =
    Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 3, 3> sensor_orientation_wrt_ref_frame_;
  Eigen::Matrix<double, 6, 1> estimated_wrench_in_sensor_frame_ =
    Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> estimated_wrench_in_interaction_frame_ =
    Eigen::Matrix<double, 6, 1>::Zero();

// Parameters
  /// mass, com, f_0 and tau_0
  FtParameters ft_calib_parameters_;
  /// Gravity in robot base (typically, [0, 0, -9.81] m.s-2)
  Eigen::Vector3d gravity_in_robot_base_frame_ = Eigen::Vector3d::Zero();
  /**
   * Transformation \f${}^{\text{sensor}}T_{\text{interaction}}\f$ from interaction frame
   * (i.e. end-effector) to sensor
   */
  Eigen::Isometry3d interaction_frame_to_sensor_frame_;
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

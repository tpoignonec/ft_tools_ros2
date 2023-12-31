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

#include "ft_tools/ft_estimation.hpp"

#include <iostream>

namespace ft_tools
{
FtEstimation::FtEstimation() {}
FtEstimation::~FtEstimation() {}

bool FtEstimation::init(
  const FtParameters & ft_calib_parameters,
  const Eigen::Matrix<double, 6, 1> & deadband,
  const Eigen::Isometry3d & interaction_frame_wrt_sensor_frame,
  const Eigen::Vector3d & gravity_in_robot_base_frame)
{
  is_initialized_ = set_parameters(
    ft_calib_parameters,
    deadband,
    interaction_frame_wrt_sensor_frame,
    gravity_in_robot_base_frame
  );
  return is_initialized_;
}

bool FtEstimation::process_raw_wrench(
  const Eigen::Matrix<double, 3, 3> & sensor_orientation,
  const Eigen::Matrix<double, 6, 1> & measured_wrench
)
{
  if (!is_initialized_) {
    return false;
  }
  // Copy wrench and current sensor orientation
  raw_wrench_in_sensor_frame_ = measured_wrench;
  sensor_orientation_wrt_ref_frame_ = sensor_orientation;
  // Compensate for gravity (+ remove offset)
  estimated_wrench_in_sensor_frame_ = remove_weight(raw_wrench_in_sensor_frame_);
  apply_wrench_deadband(estimated_wrench_in_sensor_frame_);
  // Express at interaction point (i.e. end-effector)
  estimated_wrench_in_interaction_frame_ = estimate_interaction(estimated_wrench_in_sensor_frame_);
  return true;
}

const Eigen::Matrix<double, 6, 1> & FtEstimation::get_estimated_wrench()
{
  return estimated_wrench_in_sensor_frame_;
}

const Eigen::Matrix<double, 6, 1> & FtEstimation::get_estimated_interaction_wrench()
{
  return estimated_wrench_in_interaction_frame_;
}

bool FtEstimation::set_parameters(
  const FtParameters & ft_calib_parameters,
  const Eigen::Matrix<double, 6, 1> & deadband,
  const Eigen::Isometry3d & interaction_frame_wrt_sensor_frame,
  const Eigen::Vector3d & gravity_in_robot_base_frame
)
{
  // Set calib parameters
  ft_calib_parameters_.mass = ft_calib_parameters.mass;
  ft_calib_parameters_.com = ft_calib_parameters.com;
  ft_calib_parameters_.force_offset = ft_calib_parameters.force_offset;
  ft_calib_parameters_.torque_offset = ft_calib_parameters.torque_offset;
  // Setup gravity
  gravity_in_robot_base_frame_ = gravity_in_robot_base_frame;
  // Set deadband
  deadband_ = deadband;
  // Set transformation between interaction and sensor frames
  interaction_frame_wrt_sensor_frame_ = interaction_frame_wrt_sensor_frame;
  // Debug prints
  std::cerr << "FtEstimation: mass = " << ft_calib_parameters_.mass << std::endl;
  std::cerr << "FtEstimation: com = " << ft_calib_parameters_.com.transpose() << std::endl;
  std::cerr << "FtEstimation: gravity_in_robot_base_frame = " <<
    gravity_in_robot_base_frame_.transpose() << std::endl;
  std::cerr << "FtEstimation: deadband = " << deadband_.transpose() << std::endl;
  return true;
}

bool FtEstimation::set_interaction_frame_wrt_sensor_frame(
  const Eigen::Isometry3d & interaction_frame_wrt_sensor_frame
)
{
  // Set transformation between interaction and sensor frames
  interaction_frame_wrt_sensor_frame_ = interaction_frame_wrt_sensor_frame;
  return true;
}


const FtParameters & FtEstimation::get_ft_calibration()
{
  return ft_calib_parameters_;
}

const Eigen::Matrix<double, 6, 1> & FtEstimation::get_deadband()
{
  return deadband_;
}

const Eigen::Isometry3d & FtEstimation::get_transformation_interaction_to_sensor()
{
  return interaction_frame_wrt_sensor_frame_;
}

// Internal methods

Eigen::Matrix<double, 6, 1> FtEstimation::remove_weight(
  const Eigen::Matrix<double, 6, 1> & raw_wrench_in_sensor_frame)
{
  // Compute weight in sensor frame
  Eigen::Vector3d gravity_in_sensor_frame = sensor_orientation_wrt_ref_frame_.transpose() *
    gravity_in_robot_base_frame_;
  Eigen::Vector3d forces_g = ft_calib_parameters_.mass * gravity_in_sensor_frame;
  Eigen::Vector3d torques_g = ft_calib_parameters_.com.cross(forces_g);

  // Gravity compensation
  Eigen::Matrix<double, 6, 1> estimated_wrench_in_sensor_frame = raw_wrench_in_sensor_frame;
  estimated_wrench_in_sensor_frame.head(3) -= forces_g;
  estimated_wrench_in_sensor_frame.tail(3) -= torques_g;

  // Remove static offsets
  estimated_wrench_in_sensor_frame.head(3) -= ft_calib_parameters_.force_offset;
  estimated_wrench_in_sensor_frame.tail(3) -= ft_calib_parameters_.torque_offset;

  return estimated_wrench_in_sensor_frame;
}

void FtEstimation::apply_wrench_deadband(Eigen::Matrix<double, 6, 1> & wrench)
{
  for (int i = 0; i < 6; i++) {
    if ((wrench[i] < deadband_[i]) && (wrench[i] > -deadband_[i])) {
      wrench[i] = 0;
    }
  }
}

Eigen::Matrix<double, 6, 1> FtEstimation::estimate_interaction(
  const Eigen::Matrix<double, 6, 1> & wrench_in_sensor_frame)
{
  // Compute "lever effect" (i.e. torque generated by forces at interaction point)
  Eigen::Vector3d dist_sensor_interaction = interaction_frame_wrt_sensor_frame_.translation();
  Eigen::Vector3d forces_interaction = wrench_in_sensor_frame.head(3);
  Eigen::Vector3d lever_torques = dist_sensor_interaction.cross(forces_interaction);

  // Express in interaction frame
  Eigen::Matrix<double, 6,
    1> interaction_wrench_in_interaction_frame = Eigen::Matrix<double, 6, 1>::Zero();
  interaction_wrench_in_interaction_frame.head(3) =
    interaction_frame_wrt_sensor_frame_.linear().transpose() * forces_interaction;
  interaction_wrench_in_interaction_frame.tail(3) =
    interaction_frame_wrt_sensor_frame_.linear().transpose() *
    (wrench_in_sensor_frame.tail(3) - lever_torques);

  return interaction_wrench_in_interaction_frame;
}

}  // namespace ft_tools

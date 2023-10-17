// Copyright 2022, ICube Laboratory, University of Strasbourg
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

#include "ft_tools/ft_calibration.hpp"

#include <iostream>

namespace ft_tools
{

bool FtCalibration::reset()
{
  N = 0;
  H.resize(0, 0);
  Z.resize(0);
  return true;
}

bool FtCalibration::add_measurement(
  const Eigen::Vector3d & gravity, const Eigen::Matrix<double, 6,
  1> & measured_wrench)
{
  // Define weights
  Eigen::MatrixXd weights(6, 6);
  weights.setIdentity();
  double weight_force = 3;   // Give more weight to force!
  double weight_torques = 1;
  weights.diagonal().head(3) *= weight_force;
  weights.diagonal().tail(3) *= weight_torques;

  // Build observation matrix
  Eigen::MatrixXd H_i(6, 10);
  H_i.setZero();

  Eigen::Matrix<double, 3, 3> skew_gravity;
  skew_gravity << 0.0, -gravity[2], gravity[1],
    gravity[2], 0.0, -gravity[0],
    -gravity[1], gravity[0], 0.0;

  // f_meas = -m*g + f_0
  // tau_meas = -mc x g + tau_0 = g x mc = skew_g * mc
  // Note: here, gravity already contains the sign...
  H_i.block<3, 1>(0, 0) = gravity;
  H_i.block<3, 3>(3, 1) = -skew_gravity;
  H_i.rightCols(6).setIdentity();

  // Apply weight on left side term
  H_i = weights * H_i;

  // Append observation matrix
  if (N == 0) {
    H = H_i;
    // Apply weight on right side term
    Z = weights * measured_wrench;
  } else {
    H.conservativeResize(H.rows() + 6, H.cols());
    H.bottomRows(6) = H_i;
    Z.conservativeResize(Z.rows() + 6, Z.cols());
    // Apply weight on right side term
    Z.bottomRows(6) = weights * measured_wrench;
  }
  N++;

  // std::cout << "H = " << H << std::endl;
  // std::cout << "Z = " << Z << std::endl;
  return true;
}

bool FtCalibration::get_parameters(FtParameters & ft_calib_parameters)
{
  if (N < 10) {
    // TODO(tpoignonec): error msg
    return false;
  }
  // Solve problem
  Eigen::VectorXd phi(10);
  if (!solve(phi) || phi.size() != 10) {
    return false;
    std::cerr << "Failed to solve F/T calibration problem! (N sample = " << N << ") \n";
  }

  // std::cerr << "phi= " << phi << "\n";
  // Extract parameters
  double m = phi[0];
  Eigen::Vector3d com = phi.segment<3>(1) / m;
  Eigen::Vector3d force_offset = phi.segment<3>(4);
  Eigen::Vector3d torque_offset = phi.segment<3>(7);

  ft_calib_parameters.mass = m;
  ft_calib_parameters.com = com;
  ft_calib_parameters.force_offset = force_offset;
  ft_calib_parameters.torque_offset = torque_offset;

  return true;
}

bool FtCalibration::solve(Eigen::VectorXd & phi)
{
  phi = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Z);
  return true;
}

}  // namespace ft_tools

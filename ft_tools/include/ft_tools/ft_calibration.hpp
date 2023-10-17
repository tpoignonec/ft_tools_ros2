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

#ifndef FT_TOOLS__FT_CALIBRATION_HPP_
#define FT_TOOLS__FT_CALIBRATION_HPP_

#include "ft_tools/ft_parameters.hpp"
// misc.
#include <Eigen/Core>
#include <string>
#include <memory>


namespace ft_tools
{

class FtCalibration
{
public:
  FtCalibration() {}
  ~FtCalibration() {}

  /**
  * @brief Reset FT sensor calibration estimator
  *
  */
  bool reset();

  bool add_measurement(
    const Eigen::Vector3d & gravity,
    const Eigen::Matrix<double, 6, 1> & measured_wrench);

  bool get_parameters(FtParameters & ft_calib_parameters);

  unsigned int get_number_measurements() {return N;}

protected:
  /**
   * @brief solve H @ phi = Z
   * where phi = [m, m*c_x, m*c_z, m*c_z, f_o, tau_0]. f_o and tau_0 are 3x1 vectors
   *
   * @param phi the estimated (augmented) calib parameter vector
   * @return true OK
   * @return false NOK
   */
  bool solve(Eigen::VectorXd & phi);

  unsigned int N = 0;
  Eigen::MatrixXd H;       // (stacked) observation matrices
  Eigen::VectorXd Z;       // (stacked) F/T measurements
};

}  // namespace ft_tools

#endif  // FT_TOOLS__FT_CALIBRATION_HPP_

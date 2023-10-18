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

#ifndef FT_TOOLS__FT_PARAMETERS_HPP_
#define FT_TOOLS__FT_PARAMETERS_HPP_

#include <Eigen/Dense>

#include <string>
#include <iostream>

#include "ft_msgs/msg/ft_calibration.hpp"

namespace ft_tools
{

class FtParameters
{
public:
  FtParameters();

  /// Test for parameters consistency
  bool is_valid();

  /// Print F/T calibration parameters to console
  void print_calib();

  /// Load parameters from YAML file
  bool from_yaml(const std::string & filename);

  /// Load parameters from YAML file
  bool from_yaml(const std::string & config_filename, const std::string & config_package);

  /// Dump parameters to YAML file
  bool to_yaml(const std::string & filename);

  /// Dump parameters to YAML file
  bool to_yaml(const std::string & config_filename, const std::string & config_package);

  /// Load parameters from msg
  bool from_msg(const ft_msgs::msg::FtCalibration & msg);

  /// Write parameters to msg
  ft_msgs::msg::FtCalibration to_msg();


  // F/T calibration parameters
  double mass;
  Eigen::Vector3d com;
  Eigen::Vector3d force_offset;
  Eigen::Vector3d torque_offset;
protected:
  std::string get_config_file_path(
    const std::string & config_filename,
    const std::string & config_package
  );
};

}  // namespace ft_tools

#endif  // FT_TOOLS__FT_PARAMETERS_HPP_

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

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <filesystem>
#include <fstream>
#include <iostream>

#include "ft_tools/ft_parameters.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace ft_tools
{

FtParameters::FtParameters()
{
  mass = 0.0;
  com = Eigen::Vector3d::Zero();
  force_offset = Eigen::Vector3d::Zero();
  torque_offset = Eigen::Vector3d::Zero();
}

bool FtParameters::is_valid()
{
  bool all_valid = true;
  all_valid &= mass >= 0.0;

  return all_valid;
}

void FtParameters::print_calib()
{
  std::cout << "======================================================" << std::endl;
  std::cout << "== F/T calibration parameters:" << std::endl;
  std::cout << "==   - mass = " << mass << " Kg" << std::endl;
  std::cout << "==   - com = " << com << " m" << std::endl;
  std::cout << "==   - f_0 = " << force_offset << " N" << std::endl;
  std::cout << "==   - tau_0 = " << torque_offset << " Nm" << std::endl;
  std::cout << "======================================================" << std::endl;
}

// Loader
bool FtParameters::from_yaml(const std::string & filename)
{
  YAML::Node config = YAML::LoadFile(filename);
  bool all_ok = true;
  // Extract mass
  mass = config["mass"].as<double>();
  if (mass < 0) {
    std::cerr << "Invalid F/T calibration parameter. Mass must be positive!" << std::endl;
    all_ok &= false;
  }
  // Extract vector data
  auto load_vector_3D = [&](const std::string & name_setting, Eigen::Vector3d & vector_out) -> bool
    {
      auto value_node = config[name_setting];
      if (!value_node) {
        std::cerr << \
          "Invalid F/T calibration parameter ('" \
                  << name_setting << "' does not exist!)" << std::endl;
        return false;
      }
      std::vector<double> vector_std = value_node.as<std::vector<double>>();
      if (vector_std.size() != 3) {
        std::cerr << \
          "Invalid F/T calibration parameter ('" \
                  << name_setting << "' must be a 3D vector!)" << std::endl;
        return false;
      }
      vector_out = Eigen::Map<Eigen::Vector3d>(vector_std.data(), vector_std.size());
      return true;
    };

  all_ok &= load_vector_3D("sensor_frame_to_com", com);
  all_ok &= load_vector_3D("force_offset", force_offset);
  all_ok &= load_vector_3D("torque_offset", torque_offset);

  return all_ok;
}

bool FtParameters::from_yaml(
  const std::string & config_filename,
  const std::string & config_package)
{
  return from_yaml(get_config_file_path(config_filename, config_package));
}

// Dumper
bool FtParameters::to_yaml(const std::string & filename)
{
  YAML::Node config;
  config["mass"] = mass;
  auto dump_vector = [&](const std::string & name_setting, Eigen::Vector3d vector_in) -> bool
    {
      std::vector<double> vec(vector_in.size());
      Eigen::Map<Eigen::Vector3d>(vec.data(), vec.size()) = vector_in;
      for (auto element : vector_in) {
        config[name_setting].push_back(element);
      }
      return true;
    };

  bool all_ok = true;
  all_ok &= dump_vector("sensor_frame_to_com", com);
  all_ok &= dump_vector("force_offset", force_offset);
  all_ok &= dump_vector("torque_offset", torque_offset);

  std::ofstream fout(filename);
  fout << config;

  return all_ok;
}

bool FtParameters::to_yaml(
  const std::string & config_filename,
  const std::string & config_package)
{
  return to_yaml(get_config_file_path(config_filename, config_package));
}


// Load parameters from msg
bool FtParameters::from_msg(const ft_msgs::msg::FtCalibration & msg)
{
  if (msg.mass < 0) {
    return false;
  }

  mass = msg.mass;

  com[0] = msg.sensor_frame_to_com.x;
  com[1] = msg.sensor_frame_to_com.y;
  com[2] = msg.sensor_frame_to_com.z;

  force_offset[0] = msg.force_offset.x;
  force_offset[1] = msg.force_offset.y;
  force_offset[2] = msg.force_offset.z;

  torque_offset[0] = msg.torque_offset.x;
  torque_offset[1] = msg.torque_offset.y;
  torque_offset[2] = msg.torque_offset.z;

  return true;
}

// Write parameters to msg
ft_msgs::msg::FtCalibration FtParameters::to_msg()
{
  ft_msgs::msg::FtCalibration msg;

  msg.mass = mass;

  msg.sensor_frame_to_com.x = com[0];
  msg.sensor_frame_to_com.y = com[1];
  msg.sensor_frame_to_com.z = com[2];

  msg.force_offset.x = force_offset[0];
  msg.force_offset.y = force_offset[1];
  msg.force_offset.z = force_offset[2];

  msg.torque_offset.x = torque_offset[0];
  msg.torque_offset.y = torque_offset[1];
  msg.torque_offset.z = torque_offset[2];

  return msg;
}

std::string FtParameters::get_config_file_path(
  const std::string & config_filename,
  const std::string & config_package
)
{
  std::string share_dir_path = ament_index_cpp::get_package_share_directory(config_package);
  auto config_file_path = std::filesystem::path(share_dir_path) / "config" / config_filename;
  std::string filename{config_file_path.u8string()};
  return filename;
}

}  // namespace ft_tools

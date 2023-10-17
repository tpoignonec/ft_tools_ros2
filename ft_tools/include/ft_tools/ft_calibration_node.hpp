// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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
//
// Author: Thibault Poignonec (thibault.poignonec@gmail.com)

#ifndef FT_TOOLS__FT_CALIBRATION_NODE_HPP_
#define FT_TOOLS__FT_CALIBRATION_NODE_HPP_

#include <Eigen/Core>

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"

#include "ft_tools/ft_calibration.hpp"
#include "ft_msgs/srv/add_calibration_sample.hpp"
#include "ft_msgs/srv/get_calibration.hpp"

namespace ft_tools
{

class FtCalibrationNode : public rclcpp::Node
{
public:
  FtCalibrationNode();

  bool update_parameters();

  void add_calibration_sample(
    const std::shared_ptr<ft_msgs::srv::AddCalibrationSample::Request> request,
    std::shared_ptr<ft_msgs::srv::AddCalibrationSample::Response> response);

  void get_calibration(
    const std::shared_ptr<ft_msgs::srv::GetCalibration::Request> request,
    std::shared_ptr<ft_msgs::srv::GetCalibration::Response> response);

  void save_calibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

protected:
  // Services
  rclcpp::Service<ft_msgs::srv::AddCalibrationSample>::SharedPtr srv_add_calibration_;
  rclcpp::Service<ft_msgs::srv::GetCalibration>::SharedPtr srv_get_calibration_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_calibration_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;

  // F/T calibration utils
  unsigned int min_nb_samples_;
  FtCalibration ft_calibration_process_;

  // Data
  Eigen::Vector3d gravity_in_ref_frame_;
  Eigen::Isometry3d sensor_frame_wrt_ref_frame_;
  Eigen::Matrix<double, 6, 1> raw_wrench_;
};

}  // namespace ft_tools

#endif  // FT_TOOLS__FT_CALIBRATION_NODE_HPP_

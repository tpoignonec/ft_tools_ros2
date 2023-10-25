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
#include <map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "ft_tools/ft_calibration.hpp"
#include "ft_tools/joint_state_monitor.hpp"
#include "ft_msgs/srv/add_calibration_sample.hpp"
#include "ft_msgs/srv/get_calibration.hpp"
#include "tf2_ros/transform_broadcaster.h"

// include generated parameter library
#include "ft_calibration_node_parameters.hpp"

namespace ft_tools
{

class FtCalibrationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  FtCalibrationNode();

  bool update_parameters();

  bool update_robot_state();

  void callback_new_raw_wrench(const geometry_msgs::msg::WrenchStamped & msg_raw_wrench);

  void add_calibration_sample(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

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
  bool init_kinematics_monitoring();
  bool register_services();

  // DEBUG ONLY
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_wrench_sensor_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters management
  std::shared_ptr<ft_calibration_node::ParamListener> parameter_handler_;
  ft_calibration_node::Params parameters_;

  /// Raw wrench measurements subscriber
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    raw_wrench_subscriber_{nullptr};

  /// Joint state monitor (i.e., subscriber + utils)
  ft_tools::JointStateMonitor robot_joint_state_monitor_;

  /// Index map (to deal with unordered joint states...)
  std::map<std::string, double> joint_positions_idx_;

  /// Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;

  /// Kinematics interface
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_add_calibration_;
  rclcpp::Service<ft_msgs::srv::GetCalibration>::SharedPtr srv_get_calibration_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_calibration_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;

  // F/T calibration utils
  FtCalibration ft_calibration_process_;

  // Data
  Eigen::Vector3d gravity_in_reference_frame_;
  geometry_msgs::msg::WrenchStamped msg_raw_wrench_;
  Eigen::Matrix<double, 6, 1> raw_wrench_;
  Eigen::Matrix<double, 6, 1> sensor_twist_wrt_robot_base_;

  /// Homogeneous transformation \f${}^b T_s \f$
  Eigen::Isometry3d sensor_frame_wrt_robot_base_;
  /// Homogeneous transformation \f${}^b T_r \f$
  Eigen::Isometry3d ref_frame_wrt_robot_base_;
  /// Homogeneous transformation \f${}^r T_s \f$
  Eigen::Isometry3d sensor_frame_wrt_ref_frame_;
};

}  // namespace ft_tools

#endif  // FT_TOOLS__FT_CALIBRATION_NODE_HPP_

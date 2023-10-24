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

#ifndef FT_TOOLS__FT_ESTIMATION_NODE_HPP_
#define FT_TOOLS__FT_ESTIMATION_NODE_HPP_

#include <string>
#include <memory>

#include "ft_tools/ft_estimation.hpp"
#include "ft_tools/joint_state_monitor.hpp"
#include "ft_msgs/srv/get_calibration.hpp"
#include "ft_msgs/srv/set_calibration.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "std_srvs/srv/trigger.hpp"

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"

// include generated parameter library
#include "ft_estimation_node_parameters.hpp"

namespace ft_tools
{

class FtEstimationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  FtEstimationNode();

  bool update_parameters(bool force_update = false);

  bool process_new_raw_wrench(const geometry_msgs::msg::WrenchStamped & msg_raw_wrench);

  bool update_robot_state();

  void callback_new_raw_wrench(const geometry_msgs::msg::WrenchStamped & msg_raw_wrench);

  // Service callbacks
  void set_calibration(
    const std::shared_ptr<ft_msgs::srv::SetCalibration::Request> request,
    std::shared_ptr<ft_msgs::srv::SetCalibration::Response> response);

  void get_calibration(
    const std::shared_ptr<ft_msgs::srv::GetCalibration::Request> request,
    std::shared_ptr<ft_msgs::srv::GetCalibration::Response> response);

  void save_calibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void reload_calibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

protected:
  bool register_services();

  bool init_kinematics_monitoring();

  // Parameters management
  std::shared_ptr<ft_estimation_node::ParamListener> parameter_handler_;
  ft_estimation_node::Params parameters_;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
    estimated_sensor_wrench_publisher_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
    estimated_interaction_wrench_publisher_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    raw_wrench_subscriber_{nullptr};

  // Data
  geometry_msgs::msg::WrenchStamped last_msg_raw_wrench_;
  bool is_first_wrench_ = true;
  Eigen::Matrix<double, 6, 1> last_interaction_wrench_;

  // Frames of reference

  /// Name of the reference frame \f$\mathcal{F}_r\f$
  std::string reference_frame_;
  /// Name of the sensor frame \f$\mathcal{F}_s\f$
  std::string sensor_frame_;
  /// Name of the interaction frame \f$\mathcal{F}_i\f$
  std::string interaction_frame_;

  /// Homogeneous transformation \f${}^b T_r \f$
  Eigen::Isometry3d ref_frame_wrt_robot_base_;
  /// Homogeneous transformation \f${}^b T_s \f$
  Eigen::Isometry3d sensor_frame_wrt_robot_base_;
  /// Homogeneous transformation \f${}^b T_i \f$
  Eigen::Isometry3d interaction_frame_wrt_robot_base_;

  /// Homogeneous transformation \f${}^rT_s \f$
  Eigen::Isometry3d sensor_frame_wrt_ref_frame_;
  /// Homogeneous transformation \f${}^rT_i \f$
  Eigen::Isometry3d interaction_frame_wrt_ref_frame_;
  /// Homogeneous transformation \f${}^sT_i \f$
  Eigen::Isometry3d interaction_frame_wrt_sensor_frame_;

  /// Joint state monitor (i.e., subscriber + utils)
  JointStateMonitor robot_joint_state_monitor_;

  /// Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;

  /// Kinematics interface
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  // Services
  rclcpp::Service<ft_msgs::srv::SetCalibration>::SharedPtr srv_set_calibration_;
  rclcpp::Service<ft_msgs::srv::GetCalibration>::SharedPtr srv_get_calibration_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_calibration_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reload_calibration_;

  // F/T estimation utils
  FtEstimation ft_estimation_process_;
  /// Wrench deadband in [N,N,N,Nm,Nm,Nm]
  Eigen::Matrix<double, 6, 1> wrench_deadband_;
  /// Gravity setting
  Eigen::Matrix<double, 3, 1> gravity_in_reference_frame_;
};

}  // namespace ft_tools

#endif  // FT_TOOLS__FT_ESTIMATION_NODE_HPP_

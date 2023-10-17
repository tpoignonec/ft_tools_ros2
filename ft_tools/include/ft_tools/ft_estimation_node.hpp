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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"


namespace ft_tools
{

class FtEstimationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  FtEstimationNode();

  void callback_new_raw_wrench(const geometry_msgs::msg::WrenchStamped & msg_raw_wrench);

  bool update_robot_state();

protected:
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
    estimated_sensor_wrench_publisher_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
    estimated_interaction_wrench_publisher_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    raw_wrench_subscriber_{nullptr};

  // Frames of reference
  std::string reference_frame_, sensor_frame_, interaction_frame_;

  Eigen::Isometry3d sensor_frame_wrt_robot_base_;
  Eigen::Isometry3d interaction_frame_wrt_robot_base_;
  Eigen::Isometry3d ref_frame_wrt_robot_base_;

  Eigen::Isometry3d sensor_frame_wrt_ref_frame_;
  Eigen::Isometry3d interaction_frame_wrt_sensor_frame_;

  // Joint state monitor (i.e., subscriber + utils)
  JointStateMonitor robot_joint_state_monitor_;

  // Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;

  /// Kinematics interface
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  // F/T estimation utils
  FtEstimation ft_estimation_process_;
};

}  // namespace ft_tools

#endif  // FT_TOOLS__FT_ESTIMATION_NODE_HPP_

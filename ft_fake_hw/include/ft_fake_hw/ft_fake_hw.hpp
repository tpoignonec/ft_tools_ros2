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
/// \authors: Thibault Poignonec

// Adapted from https://github.com/PickNikRobotics/topic_based_ros2_control

#pragma once

// C++
#include <memory>
#include <string>

// ROS
#include "realtime_tools/realtime_buffer.h"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/wrench.hpp>

namespace ft_fake_hw
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FtFakeHw : public hardware_interface::SensorInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  // Node and subsreiber
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_fake_values_subscriber_;
  rclcpp::Node::SharedPtr node_;

  // real-time buffer
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Wrench>>
    input_wrench_fake_values_msg_;

  /// Name of the force sensor interfaces
  std::vector<std::string> interfaces_;
  /// Wrench measurement
  std::vector<double> fake_wrench_values_;

  template <typename HandleType>
  bool getInterface(const std::string& name, const std::string& interface_name, const size_t vector_index,
                    std::vector<std::vector<double>>& values, std::vector<HandleType>& interfaces);
};

}  // namespace ft_fake_hw

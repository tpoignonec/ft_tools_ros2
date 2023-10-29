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

#include <algorithm>
#include <cmath>
#include <iterator>
// #include <limits>
// #include <set>
#include <string>
#include <vector>

#include <rclcpp/executors.hpp>

#include <ft_fake_hw/ft_fake_hw.hpp>


namespace ft_fake_hw
{

CallbackReturn FtFakeHw::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Initialize storage
  fake_wrench_values_.resize(6, 0.0);

  // Create node  and wrench subscriber
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=ft_fake_hw_" + info_.name});
  node_ = rclcpp::Node::make_shared("_", options);

  const auto get_hardware_parameter =
    [this](const std::string & parameter_name, const std::string & default_value) {
      if (auto it = info_.hardware_parameters.find(parameter_name);
        it != info_.hardware_parameters.end())
      {
        return it->second;
      }
      return default_value;
    };

  // reset wrench buffer
  input_wrench_fake_values_msg_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Wrench>>(nullptr);
  // register subscriber
  wrench_fake_values_subscriber_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
    get_hardware_parameter("topic", "/fake_wrench_data"),
    rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Wrench::SharedPtr wrench)
    {
      input_wrench_fake_values_msg_.writeFromNonRT(wrench);
    }
  );

  // Get interfaces names (empty interface = value not set later!)
  interfaces_.push_back(get_hardware_parameter("force.x.state_interface", ""));
  interfaces_.push_back(get_hardware_parameter("force.y.state_interface", ""));
  interfaces_.push_back(get_hardware_parameter("force.z.state_interface", ""));
  interfaces_.push_back(get_hardware_parameter("torque.x.state_interface", ""));
  interfaces_.push_back(get_hardware_parameter("torque.y.state_interface", ""));
  interfaces_.push_back(get_hardware_parameter("torque.z.state_interface", ""));

  if (std::all_of(
      interfaces_.begin(), interfaces_.end(),
      [this](auto const & interface) {return interface.empty();}))
  {
    // all empty
    RCLCPP_FATAL(node_->get_logger(), "You must set at least one interface!");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FtFakeHw::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(node_->get_logger(), "FtFakeHw::export_state_interfaces() called.");
  RCLCPP_INFO(node_->get_logger(), " info_.name = %s", info_.name.c_str());
  RCLCPP_INFO(node_->get_logger(), " info_.sensors.size() = %u", info_.sensors.size());

  // Sensors' state interfaces
  for (auto i = 0u; i < info_.sensors.size(); i++) {
    const auto & sensor = info_.sensors[i];
    // TODO(tpoignonec): if sensor.name != params.sensor_name, break
    RCLCPP_INFO(node_->get_logger(), "Setting up %s:", sensor.name.c_str());
    RCLCPP_INFO(
      node_->get_logger(), "sensor.state_interfaces.size() = %u", sensor.state_interfaces.size());
    for (auto j = 0u; j < sensor.state_interfaces.size(); j++) {
      auto it = std::find(
        interfaces_.begin(), interfaces_.end(), sensor.state_interfaces[j].name);
      if (it != interfaces_.end()) {
        auto index = std::distance(interfaces_.begin(), it);
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            sensor.name,
            sensor.state_interfaces[j].name,
            &fake_wrench_values_[index]
        ));
      } else {
        // Not found
        throw std::runtime_error("Invalid sensor name or (fake) f/t sensor URDF parameters!");
      }
    }
  }
  return state_interfaces;
}


hardware_interface::return_type FtFakeHw::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }

  auto wrench_values_msg = input_wrench_fake_values_msg_.readFromRT();
  if (!wrench_values_msg || !(*wrench_values_msg)) {
    auto clock = node_->get_clock();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock, 1000, "No wrench data received...");
  } else {
    fake_wrench_values_[0] = (*wrench_values_msg)->force.x;
    fake_wrench_values_[1] = (*wrench_values_msg)->force.y;
    fake_wrench_values_[2] = (*wrench_values_msg)->force.z;
    fake_wrench_values_[3] = (*wrench_values_msg)->torque.x;
    fake_wrench_values_[4] = (*wrench_values_msg)->torque.y;
    fake_wrench_values_[5] = (*wrench_values_msg)->torque.z;
  }
  return hardware_interface::return_type::OK;
}
}  // end namespace ft_fake_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ft_fake_hw::FtFakeHw, hardware_interface::SensorInterface)

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


//-----------------------------------------------------------------------------
/*!\file    joint_state_monitor.cpp
 *
 * \author  thibault Poignonec <tpoignonec@unistra.fr>
 * \date    2022/06/27
 */
//-----------------------------------------------------------------------------

#include "ft_tools/joint_state_monitor.hpp"

#include <iostream>

namespace ft_tools
{

JointStateMonitor::JointStateMonitor()
: rt_joint_state_ptr_(nullptr),
  joints_state_subscriber_(nullptr)
{
  // TODO(tpoignonec): Anything?
}

JointStateMonitor::~JointStateMonitor() {}


//------------------------------------------------------------------------------
bool JointStateMonitor::init(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_handle,
  const std::string & joint_states_topic)
{
  node_handle_ = node_handle;
  joint_states_topic_ = joint_states_topic;
  node_handle->declare_parameter<std::vector<std::string>>(
    "joint_state_monitor.joints",
    std::vector<std::string>{}
  );
  ordered_joint_names_ =
    node_handle->get_parameter("joint_state_monitor.joints").as_string_array();
  ordered_joint_index_map_.clear();
  if (!ordered_joint_names_.empty()) {
    for (size_t i = 0; i < ordered_joint_names_.size(); i++) {
      ordered_joint_index_map_.insert(std::make_pair(ordered_joint_names_[i], i));
    }
  }

  // Create (RT pipe-based) "joint states" msg subscriber
  joints_state_subscriber_ = node_handle->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      rt_joint_state_ptr_.writeFromNonRT(msg);
    });

  return true;
}

//------------------------------------------------------------------------------
bool JointStateMonitor::update()
{
  // getting the data from the subscriber using the rt pipe
  auto joint_states_msg = rt_joint_state_ptr_.readFromRT();

  // no msg received yet
  if (!joint_states_msg || !(*joint_states_msg)) {
    is_ready_ = false;
    return false;
  }
  // Local copy of the msg
  last_joint_states_msg_ = *joint_states_msg->get();
  auto nb_joints = last_joint_states_msg_.name.size();
  // get joint positions from state message
  if (ordered_joint_names_.empty()) {
    joint_state_order_.resize(nb_joints);
    for (int i = 0; i < nb_joints; ++i) {
      joint_state_order_[i] = i;
    }
    RCLCPP_WARN_THROTTLE(
      node_handle_->get_logger(), *node_handle_->get_clock(), 1000,
      "JointStateMonitor. The joint data will not be reodered (param 'joints' empty)!");
  } else {
    // Test vector size...
    // TODO(tpoignonec): ordered_joint_index_map_.size() != last_joint_states_msg_->name.size()

    // Update reordering vector
    joint_state_order_.resize(nb_joints);
    for (size_t i = 0; i < joint_state_order_.size(); i++) {
      joint_state_order_[i] = ordered_joint_index_map_[last_joint_states_msg_.name[i]];
    }
  }
  // Update status
  last_timestamp_ = last_joint_states_msg_.header.stamp;
  if (!is_ready_) {
    is_ready_ = true;
  }
  bool all_ok = is_ready_ && check_timeout(0.01);  // 10ms default timeout
  if (all_ok) {
    joint_names_ = get_reordered_vector(last_joint_states_msg_.name, joint_state_order_);
    joint_positions_ = get_reordered_vector(last_joint_states_msg_.position, joint_state_order_);
    joint_velocities_ = get_reordered_vector(last_joint_states_msg_.velocity, joint_state_order_);
    joint_torques_ = get_reordered_vector(last_joint_states_msg_.effort, joint_state_order_);
  }
  return all_ok;
}

//------------------------------------------------------------------------------

bool JointStateMonitor::get_joint_states_msg(sensor_msgs::msg::JointState & msg) const
{
  if (!is_ready_) {
    return false;
  }
  msg = last_joint_states_msg_;
  return true;
}

bool JointStateMonitor::get_joint_names(std::vector<std::string> & joint_names) const
{
  if (!is_ready_) {
    return false;
  }
  joint_names = joint_names_;
  return true;
}

bool JointStateMonitor::get_joint_positions(std::vector<double> & joint_positions) const
{
  if (!is_ready_) {
    return false;
  }
  joint_positions = joint_positions_;
  return true;
}

bool JointStateMonitor::get_joint_velocities(std::vector<double> & joint_velocities) const
{
  if (!is_ready_) {
    return false;
  }
  joint_velocities = joint_velocities_;
  return true;
}
bool JointStateMonitor::get_joint_torques(std::vector<double> & joint_torques) const
{
  if (!is_ready_) {
    return false;
  }
  joint_torques = joint_torques_;
  return true;
}

//------------------------------------------------------------------------------
// WARNING! The next three functions do not check the validity of the data...
const std::vector<std::string> & JointStateMonitor::get_joint_names() const
{
  return joint_names_;
}
const std::vector<double> & JointStateMonitor::get_joint_positions() const
{
  return joint_positions_;
}
const std::vector<double> & JointStateMonitor::get_joint_velocities() const
{
  return joint_velocities_;
}
const std::vector<double> & JointStateMonitor::get_joint_torques() const
{
  return joint_torques_;
}

template<class T>
std::vector<T> JointStateMonitor::get_reordered_vector(
  const std::vector<T> & vect,
  const std::vector<size_t> & order) const
{
  if (vect.size() != order.size()) {
    RCLCPP_FATAL(
      node_handle_->get_logger(),
      "JointStateMonitor::get_reordered_vector(). Reordering vector has invalid size!"
    );
    assert(vect.size() == order.size());
  }
  std::vector<T> reordered_vect = vect;
  for (int i = 0; i < order.size(); ++i) {
    reordered_vect[order[i]] = vect[i];
  }
  return reordered_vect;
}

template<class T>
std::vector<T> JointStateMonitor::get_reordered_vector(const std::vector<T> & vect) const
{
  return get_reordered_vector(vect, joint_state_order_);
}
//------------------------------------------------------------------------------

bool JointStateMonitor::check_timeout(double timeout /* seconds */)
{
  if (!is_ready_) {
    RCLCPP_WARN_THROTTLE(
      node_handle_->get_logger(),
      *node_handle_->get_clock(), 1000, "JointStateMonitor not initialized!!");
    return false;
  }
  double time_since_last_update_sec =
    (node_handle_->get_clock()->now().nanoseconds() - last_timestamp_.nanoseconds()) / 1e9;
  if (time_since_last_update_sec > timeout) {
    RCLCPP_WARN_THROTTLE(
      node_handle_->get_logger(), *node_handle_->get_clock(), 1000,
      "JointStateMonitor. Time since last update is %f ms for the topic '%s'",
      (float)time_since_last_update_sec * 1000,
      joint_states_topic_.c_str());
    return false;
  }
  return true;
}

}  // namespace ft_tools

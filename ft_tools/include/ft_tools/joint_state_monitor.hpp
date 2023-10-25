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

#ifndef FT_TOOLS__JOINT_STATE_MONITOR_HPP_
#define FT_TOOLS__JOINT_STATE_MONITOR_HPP_

// Misc.
#include <memory>
#include <vector>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "realtime_tools/realtime_buffer.h"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ft_tools
{

class JointStateMonitor
{
public:
  JointStateMonitor();

  ~JointStateMonitor();

  /**
   * @brief Init joint states monitor
   *
   * @param node_handle Pointer to the ros node
   * @param joint_states_topic Name of the topic to monitor
   */
  bool init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_handle,
    const std::string & joint_states_topic);

  bool update();

  bool get_joint_states_msg(sensor_msgs::msg::JointState & msg) const;

  bool get_joint_names(std::vector<std::string> & names) const;
  bool get_joint_positions(std::vector<double> & jnt_pos) const;
  bool get_joint_velocities(std::vector<double> & jnt_vel) const;
  bool get_joint_torques(std::vector<double> & jnt_torques) const;

  const std::vector<std::string> & get_joint_names() const;
  const std::vector<double> & get_joint_positions() const;
  const std::vector<double> & get_joint_velocities() const;
  const std::vector<double> & get_joint_torques() const;

  /**
   * @brief Perform a security check to ascertain that stored data is recent
   *
   * @param timeout max allowed delay between updates in seconds
   * @return True if everything is ok (initialized + no timeout)
   */
  bool check_timeout(double timeout = 0.01 /* seconds */);


  template<class T>
  std::vector<T> get_reordered_vector(
    const std::vector<T> & vect,
    const std::vector<size_t> & order) const;

  template<class T>
  std::vector<T> get_reordered_vector(const std::vector<T> & vect) const;

protected:
  // Boilerplate
  /// ros node handle to access the logger and parameter
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_handle_;
  bool is_ready_;

  // Last received data
  rclcpp::Time last_timestamp_;
  sensor_msgs::msg::JointState last_joint_states_msg_;

  // Joint state reordering
  std::vector<std::string> ordered_joint_names_;
  std::map<std::string, size_t> ordered_joint_index_map_;
  std::vector<size_t> joint_state_order_;

  // Data
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_torques_;

  // Subcriber
  std::string joint_states_topic_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>> rt_joint_state_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_state_subscriber_;
};

}  // namespace ft_tools


#endif  // FT_TOOLS__JOINT_STATE_MONITOR_HPP_

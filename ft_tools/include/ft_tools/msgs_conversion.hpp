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

#ifndef FT_TOOLS__MSGS_CONVERSION_HPP_
#define FT_TOOLS__MSGS_CONVERSION_HPP_

#include <Eigen/Core>

#include <geometry_msgs/msg/wrench_stamped.hpp>

void fill_wrench_msg(
  const Eigen::Matrix<double, 6, 1> & wrench,
  geometry_msgs::msg::WrenchStamped & msg)
{
  msg.wrench.force.x = wrench[0];
  msg.wrench.force.y = wrench[1];
  msg.wrench.force.z = wrench[2];
  msg.wrench.torque.x = wrench[3];
  msg.wrench.torque.y = wrench[4];
  msg.wrench.torque.z = wrench[5];
}

#endif  // FT_TOOLS__MSGS_CONVERSION_HPP_

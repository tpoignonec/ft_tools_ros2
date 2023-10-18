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

#include "ft_tools/ft_calibration_node.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace ft_tools
{
FtCalibrationNode::FtCalibrationNode()
: LifecycleNode("ft_calibration_node")
{
  bool all_ok = true;
  parameter_handler_ = std::make_shared<ft_calibration_node::ParamListener>(
    this->get_node_parameters_interface());
  parameters_ = parameter_handler_->get_params();

  all_ok &= update_parameters();
  ft_calibration_process_.reset();
  all_ok &= init_kinematics_monitoring();
  all_ok &= register_services();

  if (!all_ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error while initializing 'ft_calibration_node'!"
    );
    // TODO(tpoignonec): exit node (or use lifecycle management properly...)
  }
}

bool FtCalibrationNode::update_robot_state()
{
  // Read follower joint state and update kinematics
  if (!robot_joint_state_monitor_.update() || !robot_joint_state_monitor_.check_timeout()) {
    auto clock = this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 1000, "Failed to update robot joint state!");
    return false;
  }

  bool success = true;   // return flag

  const Eigen::VectorXd joint_pos = Eigen::Map<const Eigen::VectorXd>(
    robot_joint_state_monitor_.get_joint_positions().data(),
    robot_joint_state_monitor_.get_joint_positions().size()
  );
  const Eigen::VectorXd joint_vel = Eigen::Map<const Eigen::VectorXd>(
    robot_joint_state_monitor_.get_joint_velocities().data(),
    robot_joint_state_monitor_.get_joint_velocities().size()
  );

  // Update sensor pose and twist w.r.t. robot base
  success &= kinematics_->calculate_link_transform(
    joint_pos,
    parameters_.calibration.sensor_frame.id,
    sensor_frame_wrt_robot_base_
  );
  success = kinematics_->convert_joint_deltas_to_cartesian_deltas(
    joint_pos,
    joint_vel,
    parameters_.calibration.sensor_frame.id,
    sensor_twist_wrt_robot_base_
  );

  // Update reference pose w.r.t. robot base
  success &= kinematics_->calculate_link_transform(
    joint_pos,
    parameters_.calibration.reference_frame.id,
    ref_frame_wrt_robot_base_
  );

  // Update sensor pose w.r.t. robot base
  if (success) {
    sensor_frame_wrt_ref_frame_ = \
      sensor_frame_wrt_robot_base_ * ref_frame_wrt_robot_base_.inverse();
  }

  return success;
}

void FtCalibrationNode::callback_new_raw_wrench(
  const geometry_msgs::msg::WrenchStamped & msg_raw_wrench)
{
  msg_raw_wrench_ = msg_raw_wrench;
}

bool FtCalibrationNode::update_parameters()
{
  bool all_ok = true;
  // Refresh parameters
  if (parameter_handler_->is_old(parameters_)) {
    parameters_ = parameter_handler_->get_params();
  }
  // Retrieve gravity
  auto vector_gravity = parameters_.calibration.gravity_in_reference_frame;
  if (vector_gravity.size() != 3) {
    all_ok &= false;
  } else {
    gravity_in_reference_frame_[0] = vector_gravity[0];
    gravity_in_reference_frame_[1] = vector_gravity[1];
    gravity_in_reference_frame_[2] = vector_gravity[2];
  }
  return true;
}

void FtCalibrationNode::add_calibration_sample(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Adding a calibration sample..."
  );
  (void)request;
  if (!update_parameters()) {
    response->success = false;
    response->message = "Invalid parameters!";
    RCLCPP_ERROR(
      this->get_logger(),
      "add_calibration_sample() service. Invalid parameters!"
    );
    return;
  }
  rclcpp::Time now = this->get_clock()->now();

  // Extract raw wrench
  rclcpp::Time stamp_wrench = msg_raw_wrench_.header.stamp;
  double wrench_staleness = now.seconds() - stamp_wrench.seconds();
  if (wrench_staleness > 1) {
    response->success = false;
    response->message = "Stale wrench measurement!";
    RCLCPP_ERROR(
      this->get_logger(),
      "add_calibration_sample() service. Stale wrench measurement!"
    );
    return;
  }
  if (msg_raw_wrench_.header.frame_id != parameters_.calibration.sensor_frame.id) {
    // TODO(tpoignonec): if in ref frame, do the conversion...
    response->success = false;
    response->message = "Wrench should be in sensor frame!";
    RCLCPP_ERROR(
      this->get_logger(),
      "add_calibration_sample() service. Wrench should be in sensor frame!"
    );
    return;
  }
  raw_wrench_[0] = msg_raw_wrench_.wrench.force.x;
  raw_wrench_[1] = msg_raw_wrench_.wrench.force.y;
  raw_wrench_[2] = msg_raw_wrench_.wrench.force.z;
  raw_wrench_[3] = msg_raw_wrench_.wrench.torque.x;
  raw_wrench_[4] = msg_raw_wrench_.wrench.torque.y;
  raw_wrench_[5] = msg_raw_wrench_.wrench.torque.z;

  // TODO(tpoignonec): check kinematics timeout!
  bool kinematics_ok = update_robot_state();
  if (!kinematics_ok) {
    auto clock = this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 1000, "Failed to update robot kinematics!");
    response->success = false;
    response->message = "Failed to retrieve robot state or kinematics!";
    RCLCPP_ERROR(
      this->get_logger(),
      "add_calibration_sample() service. Failed to retrieve robot state or kinematics!"
    );
    return;
  }

  // Add sample
  Eigen::Vector3d gravity_in_sensor_frame =
    sensor_frame_wrt_ref_frame_.rotation().transpose() * gravity_in_reference_frame_;
  ft_calibration_process_.add_measurement(gravity_in_sensor_frame, raw_wrench_);
  // Return
  response->message = std::string("Valid samples: ") + std::to_string(
    ft_calibration_process_.get_number_measurements());
  response->success = true;
}

void FtCalibrationNode::get_calibration(
  const std::shared_ptr<ft_msgs::srv::GetCalibration::Request> request,
  std::shared_ptr<ft_msgs::srv::GetCalibration::Response> response)
{
  (void)request;
  if (ft_calibration_process_.get_number_measurements() < parameters_.calibration.min_nb_samples) {
    response->success = false;
    response->message = "Not enough measurements!";
    RCLCPP_ERROR(
      this->get_logger(),
      "get_calibration() service. Not enough measurements!"
    );
    return;
  }
  FtParameters ft_calib_parameters;
  if (!ft_calibration_process_.get_parameters(ft_calib_parameters)) {
    response->success = false;
    response->message = "Failed to compute calibration parameters!";
    RCLCPP_ERROR(
      this->get_logger(),
      "get_calibration() service. Failed to compute the calibration parameters!"
    );
  } else {
    response->ft_calibration = ft_calib_parameters.to_msg();
    response->success = true;
  }
}

void FtCalibrationNode::save_calibration(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Saving the ft_calibration..."
  );
  (void)request;
  // Refresh computed parameters
  FtParameters ft_calib_parameters;
  if (!ft_calibration_process_.get_parameters(ft_calib_parameters)) {
    response->success = false;
    response->message = "Failed to compute calibration parameters!";
    RCLCPP_ERROR(
      this->get_logger(),
      "save_calibration() service. Failed to compute calibration parameters!"
    );
    return;
  }
  // Write to yaml file
  bool success = ft_calib_parameters.to_yaml(
    parameters_.calibration.calibration_filename,
    parameters_.calibration.calibration_package
  );
  if (success) {
    response->success = true;
  } else {
    response->success = false;
    response->message = "Failed to save calibration parameters to yaml file!";
    RCLCPP_ERROR(
      this->get_logger(),
      "save_calibration() service. Failed to save calibration parameters to yaml file!"
    );
  }
}

void FtCalibrationNode::reset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  ft_calibration_process_.reset();
  if (!update_parameters()) {
    response->success = false;
    response->message = "Invalid parameters!";
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to reset the f/t calibration< Invalid parameter(s)/"
    );
    return;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Resetting the ft_calibration..."
  );
  response->success = true;
}

bool FtCalibrationNode::init_kinematics_monitoring()
{
  // Make sure it has not been initialized before
  if (kinematics_loader_ || kinematics_ || raw_wrench_subscriber_) {
    return false;
  }
  bool all_ok = true;
  // Load the differential IK plugin
  if (!parameters_.kinematics.plugin_name.empty()) {
    try {
      kinematics_loader_ =
        std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        parameters_.kinematics.plugin_package,
        "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(parameters_.kinematics.plugin_name));
      if (!kinematics_->initialize(
          this->get_node_parameters_interface(), parameters_.kinematics.tip))
      {
        all_ok &= false;
      }
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Exception while loading the IK plugin '%s': '%s'",
        parameters_.kinematics.plugin_name.c_str(), ex.what()
      );
      all_ok &= false;
    }
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "A differential IK plugin name was not specified in the config file.");
    all_ok &= false;
  }

  // Monitor follower joints state
  RCLCPP_INFO(
    this->get_logger(),
    "'topic joint state' : %s", parameters_.topic_joint_state.c_str()
  );
  all_ok &= robot_joint_state_monitor_.init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode>(this),
    parameters_.topic_joint_state);

  if (all_ok) {
    // Create raw wrench subscriber
    raw_wrench_subscriber_ =
      this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      parameters_.topic_raw_wrench,
      2,
      std::bind(&FtCalibrationNode::callback_new_raw_wrench, this, _1));
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to init the wrench estimator! Make sure the config and urdf files are correct.");
  }
  return all_ok;
}

bool FtCalibrationNode::register_services()
{
  // Make sure it has not been initialized before
  if (srv_add_calibration_ || srv_get_calibration_ || srv_save_calibration_ || srv_reset_) {
    return false;
  }
  bool all_ok = true;
  // Register services
  std::string node_name = this->get_name();
  srv_add_calibration_ = this->create_service<std_srvs::srv::Trigger>(
    node_name + "/add_calibration_sample",
    std::bind(&FtCalibrationNode::add_calibration_sample, this, _1, _2)
  );
  srv_get_calibration_ = this->create_service<ft_msgs::srv::GetCalibration>(
    node_name + "/get_calibration",
    std::bind(&FtCalibrationNode::get_calibration, this, _1, _2)
  );
  srv_save_calibration_ = this->create_service<std_srvs::srv::Trigger>(
    node_name + "/save_calibration",
    std::bind(&FtCalibrationNode::save_calibration, this, _1, _2)
  );
  srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
    node_name + "/reset",
    std::bind(&FtCalibrationNode::reset, this, _1, _2)
  );
  return all_ok;
}

}  // namespace ft_tools

// -----------------------------------------------------------------
//                             main()
// -----------------------------------------------------------------

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<ft_tools::FtCalibrationNode>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

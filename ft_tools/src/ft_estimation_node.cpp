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

#include "ft_tools/ft_estimation_node.hpp"
#include "ft_tools/msgs_conversion.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace ft_tools
{

FtEstimationNode::FtEstimationNode()
: LifecycleNode("ft_estimation_node")
{
  bool all_ok = true;
  parameter_handler_ = std::make_shared<ft_estimation_node::ParamListener>(
    this->get_node_parameters_interface()
  );
  parameters_ = parameter_handler_->get_params();
  all_ok &= update_parameters();
  all_ok &= init_kinematics_monitoring();

  // Retrieve reference, sensor, and interaction frames
  reference_frame_ = parameters_.calibration.reference_frame.id;
  RCLCPP_INFO(this->get_logger(), "'reference frame' : %s", reference_frame_.c_str());
  sensor_frame_ = parameters_.calibration.sensor_frame.id;
  RCLCPP_INFO(this->get_logger(), "'sensor frame' : %s", sensor_frame_.c_str());
  interaction_frame_ = parameters_.estimation.interaction_frame.id;
  RCLCPP_INFO(this->get_logger(), "'interaction frame' : %s", interaction_frame_.c_str());
  if (interaction_frame_.empty()) {
    interaction_frame_ = sensor_frame_;
  }

  // Init wrench estimator
  FtParameters ft_calib_parameters;
  all_ok &= ft_calib_parameters.from_yaml(
    parameters_.calibration.calibration_filename,
    parameters_.calibration.calibration_package
  );
  interaction_frame_wrt_sensor_frame_.setIdentity();
  all_ok &= ft_estimation_process_.init(
    ft_calib_parameters,
    wrench_deadband_,
    interaction_frame_wrt_sensor_frame_
  );

  // Register services
  all_ok &= register_services();

  // Register subscribers
  if (all_ok) {
    // Create estimated wrench publisher(s)
    estimated_sensor_wrench_publisher_ =
      this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      parameters_.topic_estimated_wrench, 2
      );
    estimated_interaction_wrench_publisher_ =
      this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      parameters_.topic_interaction_wrench, 2
      );
    // Create raw wrench subscriber
    raw_wrench_subscriber_ =
      this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      parameters_.topic_raw_wrench,
      2,
      std::bind(&FtEstimationNode::callback_new_raw_wrench, this, _1)
      );
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to init the wrench estimator! Make sure the config and urdf files are correct.");
  }
}

bool FtEstimationNode::update_parameters()
{
  bool all_ok = true;
  if (parameter_handler_->is_old(parameters_)) {
    // Refresh parameters
    parameters_ = parameter_handler_->get_params();
    // Update deadband
    std::vector<double> param_deadband = parameters_.estimation.wrench_deadband;
    if (!param_deadband.empty() && param_deadband.size() != 6) {
      RCLCPP_ERROR(this->get_logger(), "Invalid parameters 'deadband'");
      all_ok = false;
    } else if (param_deadband.size() == 6) {
      wrench_deadband_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(param_deadband.data());
    }
  }
  if (!all_ok) {
    RCLCPP_WARN(
      this->get_logger(),
      "The node parameters update failed!");
  }
  return all_ok;
}


bool FtEstimationNode::update_robot_state()
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
    sensor_frame_,
    sensor_frame_wrt_robot_base_
  );
  // Update interaction frame w.r.t. robot base
  success &= kinematics_->calculate_link_transform(
    joint_pos,
    interaction_frame_,
    interaction_frame_wrt_robot_base_
  );
  // Update reference pose w.r.t. robot base
  success &= kinematics_->calculate_link_transform(
    joint_pos,
    reference_frame_,
    ref_frame_wrt_robot_base_
  );

  // Update sensor pose w.r.t. robot base
  if (success) {
    sensor_frame_wrt_ref_frame_ = \
      ref_frame_wrt_robot_base_.inverse() * sensor_frame_wrt_robot_base_;
    interaction_frame_wrt_ref_frame_ = \
      ref_frame_wrt_robot_base_.inverse() * interaction_frame_wrt_robot_base_;
    interaction_frame_wrt_sensor_frame_ = \
      ref_frame_wrt_robot_base_.inverse() * interaction_frame_wrt_robot_base_;
  }

  return success;
}


void FtEstimationNode::callback_new_raw_wrench(
  const geometry_msgs::msg::WrenchStamped & msg_raw_wrench)
{
  if (!update_parameters()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Abort estimation, could not load the parameters!");
    return;
  }

  // TODO(tpoignonec): check kinematics timeout!
  bool kinematics_ok = update_robot_state();    // temporary...
  kinematics_ok &= ft_estimation_process_.set_interaction_frame_to_sensor_frame(
    interaction_frame_wrt_sensor_frame_
  );

  if (!kinematics_ok) {
    auto clock = this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 1000, "Failed to update robot kinematics!");
    return;
  }

  // Extract raw wrench
  Eigen::Matrix<double, 6, 1> raw_wrench;
  raw_wrench[0] = msg_raw_wrench.wrench.force.x;
  raw_wrench[1] = msg_raw_wrench.wrench.force.y;
  raw_wrench[2] = msg_raw_wrench.wrench.force.z;
  raw_wrench[3] = msg_raw_wrench.wrench.torque.x;
  raw_wrench[4] = msg_raw_wrench.wrench.torque.y;
  raw_wrench[5] = msg_raw_wrench.wrench.torque.z;

  // Estimate sensor and interaction wrenches
  ft_estimation_process_.process_raw_wrench(
    sensor_frame_wrt_ref_frame_.rotation(),
    raw_wrench
  );

  // Retrieve and publish estimated wrenches
  Eigen::Matrix<double, 6, 1> estimated_wrench_in_sensor_frame =
    ft_estimation_process_.get_estimated_wrench();

  geometry_msgs::msg::WrenchStamped msg_wrench;
  msg_wrench.header.frame_id = sensor_frame_;
  msg_wrench.header.stamp = msg_raw_wrench.header.stamp;
  fill_wrench_msg(estimated_wrench_in_sensor_frame, msg_wrench);
  estimated_sensor_wrench_publisher_->publish(msg_wrench);

  Eigen::Matrix<double, 6, 1> estimated_interaction_wrench_in_interaction_frame =
    ft_estimation_process_.get_estimated_interaction_wrench();
  // Express in reference frame
  Eigen::Matrix<double, 6, 1> estimated_interaction_wrench_in_reference_frame;
  estimated_interaction_wrench_in_reference_frame.head(3) = \
    interaction_frame_wrt_ref_frame_.rotation() * \
    estimated_interaction_wrench_in_interaction_frame.head(3);
  estimated_interaction_wrench_in_reference_frame.tail(3) = \
    interaction_frame_wrt_ref_frame_.rotation() * \
    estimated_interaction_wrench_in_interaction_frame.tail(3);

  // Publish interaction wrench
  geometry_msgs::msg::WrenchStamped msg_interaction_wrench;
  fill_wrench_msg(estimated_interaction_wrench_in_reference_frame, msg_interaction_wrench);
  msg_interaction_wrench.header.frame_id = reference_frame_;
  msg_interaction_wrench.header.stamp = msg_raw_wrench.header.stamp;
  estimated_interaction_wrench_publisher_->publish(msg_interaction_wrench);
}

bool FtEstimationNode::init_kinematics_monitoring()
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
      std::bind(&FtEstimationNode::callback_new_raw_wrench, this, _1));
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to init the wrench estimator! Make sure the config and urdf files are correct.");
  }
  return all_ok;
}

bool FtEstimationNode::register_services()
{
  // Make sure it has not been initialized before
  if (
    srv_get_calibration_ || \
    srv_set_calibration_ || \
    srv_save_calibration_ || \
    srv_reload_calibration_)
  {
    return false;
  }
  bool all_ok = true;
  // Register services
  std::string node_name = this->get_name();
  srv_get_calibration_ = this->create_service<ft_msgs::srv::GetCalibration>(
    node_name + "/get_calibration",
    std::bind(&FtEstimationNode::get_calibration, this, _1, _2)
  );
  srv_set_calibration_ = this->create_service<ft_msgs::srv::SetCalibration>(
    node_name + "/set_calibration",
    std::bind(&FtEstimationNode::set_calibration, this, _1, _2)
  );
  srv_save_calibration_ = this->create_service<std_srvs::srv::Trigger>(
    node_name + "/save_calibration",
    std::bind(&FtEstimationNode::save_calibration, this, _1, _2)
  );
  srv_reload_calibration_ = this->create_service<std_srvs::srv::Trigger>(
    node_name + "/reload_calibration",
    std::bind(&FtEstimationNode::reload_calibration, this, _1, _2)
  );
  return all_ok;
}

// Service callbacks
void FtEstimationNode::set_calibration(
  const std::shared_ptr<ft_msgs::srv::SetCalibration::Request> request,
  std::shared_ptr<ft_msgs::srv::SetCalibration::Response> response)
{
  FtParameters ft_calib_parameters;
  bool all_ok = ft_calib_parameters.from_msg(request->ft_calibration);
  if (!update_parameters()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Abort set_calibration(), could not load the parameters!");
  } else {
    all_ok &= ft_estimation_process_.set_parameters(
      ft_calib_parameters,
      wrench_deadband_,
      interaction_frame_wrt_sensor_frame_
    );
  }
  // Return response
  response->success = all_ok;
}

void FtEstimationNode::get_calibration(
  const std::shared_ptr<ft_msgs::srv::GetCalibration::Request> request,
  std::shared_ptr<ft_msgs::srv::GetCalibration::Response> response)
{
  FtParameters ft_calib_parameters = ft_estimation_process_.get_ft_calibration();
  response->ft_calibration = ft_calib_parameters.to_msg();
  response->success = true;
}

void FtEstimationNode::save_calibration(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Saving the ft_calibration..."
  );
  (void)request;
  bool all_ok = true;
  // Retrieve parameters
  FtParameters ft_calib_parameters = ft_estimation_process_.get_ft_calibration();
  // Write to yaml file
  if (!update_parameters()) {
    all_ok = false;
    RCLCPP_ERROR(
      this->get_logger(),
      "Abort save_calibration(), could not load the parameters!");
  } else {
    all_ok = ft_calib_parameters.to_yaml(
      parameters_.calibration.calibration_filename,
      parameters_.calibration.calibration_package
    );
  }
  // Return response
  if (all_ok) {
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

void FtEstimationNode::reload_calibration(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool all_ok = true;
  FtParameters ft_calib_parameters;
  if (!update_parameters()) {
    all_ok = false;
    RCLCPP_ERROR(
      this->get_logger(),
      "Abort save_calibration(), could not load the parameters!");
  } else {
    all_ok &= ft_calib_parameters.from_yaml(
      parameters_.calibration.calibration_filename,
      parameters_.calibration.calibration_package
    );
    all_ok &= ft_estimation_process_.set_parameters(
      ft_calib_parameters,
      wrench_deadband_,
      interaction_frame_wrt_sensor_frame_
    );
  }
  // Return response
  if (all_ok) {
    response->success = true;
  } else {
    response->success = false;
    response->message = "Failed to load calibration parameters from yaml file!";
    RCLCPP_ERROR(
      this->get_logger(),
      "reload_calibration() service. Failed to load calibration parameters from yaml file!"
    );
  }
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
    std::make_shared<ft_tools::FtEstimationNode>()->get_node_base_interface()
  );
  rclcpp::shutdown();
  return 0;
}

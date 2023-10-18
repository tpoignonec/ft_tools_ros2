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

#include "ft_tools/ft_calibration_srv_server.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace ft_tools
{
FtCalibrationSrvServer::FtCalibrationSrvServer()
: Node("ft_calibration_srv_server")
{
  this->declare_parameter<int>("min_nb_samples", 10);
  update_parameters();
  ft_calibration_process_.reset();
  // Register services
  srv_add_calibration_ = this->create_service<ft_msgs::srv::AddCalibrationSample>(
    "add_calibration_sample",
    std::bind(&FtCalibrationSrvServer::add_calibration_sample, this, _1, _2)
  );
  srv_get_calibration_ = this->create_service<ft_msgs::srv::GetCalibration>(
    "get_calibration",
    std::bind(&FtCalibrationSrvServer::get_calibration, this, _1, _2)
  );
  srv_save_calibration_ = this->create_service<std_srvs::srv::Trigger>(
    "save_calibration",
    std::bind(&FtCalibrationSrvServer::save_calibration, this, _1, _2)
  );
  srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&FtCalibrationSrvServer::reset, this, _1, _2)
  );
}

bool FtCalibrationSrvServer::update_parameters()
{
  min_nb_samples_ = this->get_parameter("min_nb_samples").as_int();
  return min_nb_samples_ >= 5;
}

void FtCalibrationSrvServer::add_calibration_sample(
  const std::shared_ptr<ft_msgs::srv::AddCalibrationSample::Request> request,
  std::shared_ptr<ft_msgs::srv::AddCalibrationSample::Response> response)
{
  if (!update_parameters()) {
    response->success = false;
    response->message = "Invalid parameters!";
    return;
  }
  // Copy wrench
  raw_wrench_[0] = request->raw_wrench.force.x;
  raw_wrench_[1] = request->raw_wrench.force.y;
  raw_wrench_[2] = request->raw_wrench.force.z;
  raw_wrench_[3] = request->raw_wrench.torque.x;
  raw_wrench_[4] = request->raw_wrench.torque.y;
  raw_wrench_[5] = request->raw_wrench.torque.z;
  // Copy sensor pose
  tf2::fromMsg(request->sensor_frame_wrt_ref_frame, sensor_frame_wrt_ref_frame_);
  // Copy gravity
  gravity_in_ref_frame_[0] = request->gravity_in_ref_frame.x;
  gravity_in_ref_frame_[1] = request->gravity_in_ref_frame.y;
  gravity_in_ref_frame_[2] = request->gravity_in_ref_frame.z;
  if (gravity_in_ref_frame_.norm() < 0.1) {
    response->success = false;
    response->message = "Srv field 'gravity_in_ref_frame' not set!";
    return;
  }
  // Add sample
  Eigen::Vector3d gravity_in_sensor_frame =
    sensor_frame_wrt_ref_frame_.rotation().transpose() * gravity_in_ref_frame_;
  ft_calibration_process_.add_measurement(gravity_in_sensor_frame, raw_wrench_);
  // Return
  response->nb_sample = ft_calibration_process_.get_number_measurements();
  response->success = true;
}

void FtCalibrationSrvServer::get_calibration(
  const std::shared_ptr<ft_msgs::srv::GetCalibration::Request> request,
  std::shared_ptr<ft_msgs::srv::GetCalibration::Response> response)
{
  (void)request;
  if (ft_calibration_process_.get_number_measurements() < min_nb_samples_) {
    response->success = false;
    response->message = "Not enough measurements!";
    return;
  }
  FtParameters ft_calib_parameters;
  if (!ft_calibration_process_.get_parameters(ft_calib_parameters)) {
    response->success = false;
    response->message = "Failed to compute calibration parameters!";
  } else {
    response->ft_calibration = ft_calib_parameters.to_msg();
    response->success = true;
  }
}

void FtCalibrationSrvServer::save_calibration(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  response->success = false;
  response->message = "Not yet implemented, sorry!";
}

void FtCalibrationSrvServer::reset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  ft_calibration_process_.reset();
  if (!update_parameters()) {
    response->success = false;
    response->message = "Invalid parameters!";
    return;
  }
  response->success = true;
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
    std::make_shared<ft_tools::FtCalibrationSrvServer>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

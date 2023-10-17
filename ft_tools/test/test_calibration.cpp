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

#include <gtest/gtest.h>
#include <Eigen/Core>

#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>

#include <ft_tools/ft_calibration.hpp>
#include <ft_tools/ft_parameters.hpp>
#include <test_asset_calibration_params.hpp>

class DataGenerator
{
public:
  explicit DataGenerator(const ft_tools::FtParameters & mock_calib)
  {
    mock_calib_ = mock_calib;
  }
  bool get_random_sample(Eigen::Vector3d & gravity, Eigen::Matrix<double, 6, 1> & measured_wrench)
  {
    // Generate random sensor orientation
    Eigen::Quaterniond quat_rand = Eigen::Quaterniond::UnitRandom();
    Eigen::MatrixXd R_sensor_frame_wrt_ref_frame = quat_rand.toRotationMatrix();

    // Generate gravity in sensor frame
    Eigen::Vector3d gravity_in_ref_frame = (Eigen::Vector3d() << 0.0, 0.0, -9.81).finished();
    gravity = R_sensor_frame_wrt_ref_frame.transpose() * gravity_in_ref_frame;

    // Generate mock measured wrench in sensor frame
    measured_wrench.head(3) = mock_calib_.mass * gravity + mock_calib_.force_offset;
    // Generate mock measured torque in sensor frame
    measured_wrench.tail(3) = \
      mock_calib_.com.cross(mock_calib_.mass * gravity) + mock_calib_.torque_offset;
    return true;
  }

// Attributes
  ft_tools::FtParameters mock_calib_;
};

TEST(TestCalibration, test_load_yaml)
{
  bool all_ok = true;
  std::string file_calib_params = "test_asset_calibration_params.yaml";

  std::ofstream out(file_calib_params);
  out << string_parameters_yaml;
  out.close();

  ft_tools::FtParameters test_calib;
  all_ok &= test_calib.from_yaml(file_calib_params);

  ft_tools::FtParameters real_calib;
  real_calib.mass = 1.2;
  real_calib.com << 0.1, 0.05, 0.05;
  real_calib.force_offset << 1.0, -1.0, 0.5;
  real_calib.torque_offset << 0.1, -0.2, 0.0;

  ASSERT_TRUE(test_calib.mass == real_calib.mass);
  ASSERT_TRUE(test_calib.com == real_calib.com);
  ASSERT_TRUE(test_calib.force_offset == real_calib.force_offset);
  ASSERT_TRUE(test_calib.torque_offset == real_calib.torque_offset);

  // Blanket check...
  ASSERT_TRUE(all_ok);
}

TEST(TestCalibration, test_dump_yaml)
{
  bool all_ok = true;
  ft_tools::FtParameters real_calib;
  real_calib.mass = 0.2;
  real_calib.com << 0.3, 0.1, 0.2;
  real_calib.force_offset << 0.0, 1.0, 2.0;
  real_calib.torque_offset << 0.5, 0.0, 1.0;

  std::string file_calib_params = "tmp_calib_params.yaml";
  all_ok &= real_calib.to_yaml(file_calib_params);

  ft_tools::FtParameters test_calib;
  all_ok &= test_calib.from_yaml(file_calib_params);

  ASSERT_TRUE(test_calib.mass == real_calib.mass);
  ASSERT_TRUE(test_calib.com == real_calib.com);
  ASSERT_TRUE(test_calib.force_offset == real_calib.force_offset);
  ASSERT_TRUE(test_calib.torque_offset == real_calib.torque_offset);

  // Blanket check...
  ASSERT_TRUE(all_ok);
}

TEST(TestCalibration, test_data_generator)
{
  ft_tools::FtParameters mock_calib;
  mock_calib.mass = 0.2;
  mock_calib.com << 0.3, 0.1, 0.2;
  mock_calib.force_offset << 0.0, 1.0, 2.0;
  mock_calib.torque_offset << 0.5, 0.0, 1.0;
  DataGenerator data_generator(mock_calib);
  Eigen::Vector3d gravity_sample;
  Eigen::Matrix<double, 6, 1> measured_wrench_sample;
  ASSERT_EQ(
    data_generator.get_random_sample(gravity_sample, measured_wrench_sample),
    true
  );
}

TEST(TestCalibration, test_calibration_logic)
{
  ft_tools::FtParameters mock_calib;
  mock_calib.mass = 0.2;
  mock_calib.com << 0.3, 0.1, 0.2;
  mock_calib.force_offset << 0.0, 1.0, 2.0;
  mock_calib.torque_offset << 0.5, 0.0, 1.0;
  DataGenerator data_generator(mock_calib);

  // Test calib process
  bool all_ok = true;
  ft_tools::FtCalibration ft_calibration_process;
  all_ok &= ft_calibration_process.reset();
  unsigned int min_N = 10;

  Eigen::Vector3d gravity_sample;
  Eigen::Matrix<double, 6, 1> measured_wrench_sample;
  for (unsigned int i = 1; i <= min_N; i++) {
    all_ok &= data_generator.get_random_sample(gravity_sample, measured_wrench_sample);
    all_ok &= ft_calibration_process.add_measurement(gravity_sample, measured_wrench_sample);
    ASSERT_EQ(ft_calibration_process.get_number_measurements(), i);
  }
  ASSERT_EQ(ft_calibration_process.get_number_measurements(), min_N);

  ft_tools::FtParameters calib_results;
  all_ok &= ft_calibration_process.get_parameters(calib_results);

  // Check estimated parameters are OK
  double default_tol = 0.001;
  double err_mass = abs(mock_calib.mass - calib_results.mass);
  ASSERT_LT(err_mass, default_tol);
  double err_com = (mock_calib.com - calib_results.com).norm();
  ASSERT_LT(err_com, default_tol);
  double err_f0 = (mock_calib.force_offset - calib_results.force_offset).norm();
  ASSERT_LT(err_f0, default_tol);
  double err_t0 = (mock_calib.torque_offset - calib_results.torque_offset).norm();
  ASSERT_LT(err_t0, default_tol);

  // Blanket check...
  ASSERT_TRUE(all_ok);
}

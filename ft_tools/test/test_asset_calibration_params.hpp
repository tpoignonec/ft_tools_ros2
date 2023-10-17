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

#ifndef TEST_ASSET_CALIBRATION_PARAMS_HPP_
#define TEST_ASSET_CALIBRATION_PARAMS_HPP_

#include <string>

const auto string_parameters_yaml =
  R"(
mass: 1.2
sensor_frame_to_com: [0.1, 0.05, 0.05]
force_offset: [1.0, -1.0, 0.5]
torque_offset: [0.1, -0.2, 0.0]
)";

#endif  // TEST_ASSET_CALIBRATION_PARAMS_HPP_

// Copyright 2024 Damien SIX (damien@robotsix.net)
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

#pragma once

#include <uav_cpp/types/setpoints.hpp>
#include <ros2_uav_interfaces/msg/pose_heading.hpp>

namespace uav_ros2::utils
{
/**
 * @brief Converts a ros2_uav_interfaces/PoseHeading message to a uavcpp::types::PoseHeading.
 * @param msg The PoseHeading message to convert.
 * @return The converted PoseHeading type.
 */
uav_cpp::types::PoseHeading convertToSetpoint(const ros2_uav_interfaces::msg::PoseHeading & msg)
{
  uav_cpp::types::PoseHeading setpoint;
  setpoint.frame_id = msg.header.frame_id;
  setpoint.position = tf2::Vector3(msg.position.x, msg.position.y, msg.position.z);
  setpoint.heading = msg.heading;
  return setpoint;
}
}  // namespace uav_ros2::utils

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

#include <uav_cpp/custom_pipelines/se3_position.hpp>
#include <uav_cpp/utils/smart_pointer_base.hpp>
#include "ros2_uav_px4/modes/attitude_thrust.hpp"

namespace ros2_uav::modes
{
/**
 * @brief Position mode class for UAV control, wrapping the uavcpp::pipelines::Se3Position class.
 */
class Position : public ros2_uav::modes::AttitudeThrustMode<uav_cpp::pipelines::Se3Position>,
  public uav_cpp::utils::SmartPointerBase<Position>
{
public:
  /**
   * @brief Constructs a new Position object.
   *
   * @param node Reference to the ROS2 node.
   */
  explicit Position(rclcpp::Node & node)
  : AttitudeThrustMode<uav_cpp::pipelines::Se3Position>(ModeBase::Settings{"Offboard Position",
        true},
      node)
  {
  }
};

}  // namespace ros2_uav::modes

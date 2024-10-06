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

#include <uav_cpp/custom_pipelines/nlmpc_waypoints.hpp>
#include <uav_cpp/utils/smart_pointer_base.hpp>
#include "ros2_uav_px4/modes/rates_thrust.hpp"

namespace ros2_uav::modes
{
/**
 * @brief Waypoints mode class for UAV control, wrapping the NlmpcWaypoints class.
 */
class NlmpcWaypoints : public ros2_uav::modes::RatesThrustMode<uav_cpp::pipelines::NlmpcWaypoints>,
  public uav_cpp::utils::SmartPointerBase<NlmpcWaypoints>
{
public:
  /**
   * @brief Constructs a new NlmpcWaypoints object.
   *
   * @param node Reference to the ROS2 node.
   */
  explicit NlmpcWaypoints(rclcpp::Node & node)
  : RatesThrustMode<uav_cpp::pipelines::NlmpcWaypoints>(
      ModeBase::Settings{"Offboard Waypoints NLMPC",
        true},
      node)
  {
  }
};
}  // namespace ros2_uav::modes

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

#include <uav_cpp/parameters/param_container.hpp>
#include <px4_ros2/components/mode.hpp>

namespace ros2_uav::modes
{
using uav_cpp::parameters::ParamContainer;
using px4_ros2::ModeBase;

/**
 * @brief Interface class for UAV modes, inheriting from ModeBase and ParamContainer.
 */
class ModeInterface : public ModeBase, public ParamContainer
{
public:
  /**
   * @brief Constructs a new ModeInterface object.
   *
   * @param mode_settings Settings for the mode.
   * @param node Reference to the ROS2 node.
   */
  ModeInterface(const ModeBase::Settings & mode_settings, rclcpp::Node & node)
  : ModeBase(node, mode_settings),
    ParamContainer(),
    node_(node)
  {
  }

protected:
  rclcpp::Node & node_;  ///< Reference to the ROS2 node.
};

}  // namespace ros2_uav::modes

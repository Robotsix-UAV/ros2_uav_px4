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

#include <memory>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include "ros2_uav_px4/mode_interface/mode_interface.hpp"

namespace ros2_uav::modes
{
using uav_cpp::types::RatesThrust;
using uav_cpp::parameters::ParamContainer;
using uav_cpp::parameters::ParameterMap;

/**
 * @brief Concept that checks if ModeT is derived from Mode with a controller that uses AttitudeThrust control.
 */
template<typename ModeT>
concept DerivedFromRatesThrustMode = requires {
  DerivedFromUavCppMode<ModeT>;
  std::is_same_v<typename ModeT::OutputType, RatesThrust>;
};

/**
 * @brief A mode class that uses attitude and thrust control.
 *
 * @tparam ModeT The mode type derived from Mode.
 */
template<DerivedFromRatesThrustMode ModeT>
class RatesThrustMode : public ModeInterface<ModeT>
{
public:
  using ModeInterface<ModeT>::addRequiredParameter;
  /**
   * @brief Constructs a new AttitudeThrustMode object.
   *
   * @param mode_settings Settings for the mode.
   * @param node Reference to the ROS2 node.
   */
  RatesThrustMode(const ModeBase::Settings & mode_settings, rclcpp::Node & node);

private:
  /**
   * @brief Updates the setpoint.
   *
   * @param dt Time delta since the last update.
   */
  void updateSetpoint([[maybe_unused]] float dt) override;

  ParameterMap::SharedPtr parameters_;  ///< Shared pointer to the parameter map.
  rclcpp::Time time_init_;  ///< Initialization time.
  std::shared_ptr<px4_ros2::RatesSetpointType> rates_setpoint_;
  ///< Shared pointer to attitude setpoint.
};

}  // namespace ros2_uav::modes

#include "ros2_uav_px4/modes/rates_thrust_impl.hpp"

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
#include <uav_cpp/modes/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include "ros2_uav_px4/mode_interface/mode_interface.hpp"

namespace ros2_uav::modes
{
using uav_cpp::modes::Mode;
using uav_cpp::types::AttitudeThrust;
using uav_cpp::parameters::ParamContainer;
using uav_cpp::parameters::ParameterMap;

/**
 * @brief Concept that checks if ModeT is derived from Mode with a controller that uses AttitudeThrust control.
 */
template<typename ModeT>
concept DerivedFromAtittudeThrustMode = requires {
  std::is_base_of_v<Mode<typename ModeT::TrackerType, typename ModeT::ControllerType>, ModeT>;
  std::is_same_v<typename ModeT::ControllerType::ControlInputsType, AttitudeThrust>;
};

/**
 * @brief A mode class that uses attitude and thrust control.
 *
 * @tparam ModeT The mode type derived from Mode.
 */
template<DerivedFromAtittudeThrustMode ModeT>
class AttitudeThrustMode : public ModeInterface
{
public:
  /**
   * @brief Constructs a new AttitudeThrustMode object.
   *
   * @param mode_settings Settings for the mode.
   * @param node Reference to the ROS2 node.
   */
  AttitudeThrustMode(const ModeBase::Settings & mode_settings, rclcpp::Node & node);

  /**
   * @brief Sets the setpoint for the mode.
   *
   * @param setpoint The setpoint to be set.
   */
  void setSetpoint(const ModeT::TrackerType::SetPointType & setpoint) {mode_.setSetpoint(setpoint);}

private:
  ModeT mode_;  ///< The mode instance.
  ParameterMap::SharedPtr parameters_;  ///< Shared pointer to the parameter map.
  rclcpp::Time time_init_;  ///< Initialization time.
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
  ///< Shared pointer to attitude setpoint.
  std::shared_ptr<px4_ros2::OdometryLocalPosition> vehicle_local_position_;
  ///< Shared pointer to vehicle local position.
  std::shared_ptr<px4_ros2::OdometryAngularVelocity> vehicle_angular_velocity_;
  ///< Shared pointer to vehicle angular velocity.
  std::shared_ptr<px4_ros2::OdometryAttitude> vehicle_attitude_;
  ///< Shared pointer to vehicle attitude.

  /**
   * @brief Function called when the mode is activated.
   */
  void onActivate() override {odometryUpdate(); mode_.reset();}

  /**
   * @brief Function called when the mode is deactivated.
   */
  void onDeactivate() override {}

  /**
   * @brief Updates the odometry data.
   */
  void odometryUpdate();

  /**
   * @brief Updates the setpoint.
   *
   * @param dt Time delta since the last update.
   */
  void updateSetpoint([[maybe_unused]] float dt) override;
};

}  // namespace ros2_uav::modes

#include "ros2_uav_px4/modes/attitude_thrust_impl.hpp"

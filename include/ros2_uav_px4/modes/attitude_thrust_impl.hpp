// Copyright 2024 The Technology Innovation Institute (TII)
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

/**
 * @author Damien SIX (damien@robotsix.net)
 */

#pragma once

#include <memory>
#include "ros2_uav_px4/utils/debug.hpp"

namespace ros2_uav::modes
{
template<DerivedFromAttitudeThrustMode PipelineT>
AttitudeThrustMode<PipelineT>::AttitudeThrustMode(
  const ModeBase::Settings & mode_settings,
  rclcpp::Node & node)
: ModeInterface<PipelineT>(mode_settings, node)
{
  this->setSetpointUpdateRate(250.0);
  attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
  px4_ros2::Result result = px4_ros2::Result::Success;
  this->completed(result);
}

template<DerivedFromAttitudeThrustMode PipelineT>
void AttitudeThrustMode<PipelineT>::updateSetpoint([[maybe_unused]] float dt)
{
  this->odometryUpdate();
  auto time_now = (this->node_.now()).nanoseconds();
  auto control_inputs = this->pipeline_->execute(std::chrono::nanoseconds(time_now));
  // Debug each module input/output
  // ros2_uav::debug::moduleLoop<0, PipelineT>(this->pipeline_);

  // Set the attitude setpoint
  const Eigen::Vector3f thrust_sp{0.0f, 0.0f, static_cast<float>(-control_inputs.thrust)};
  const Eigen::Quaternionf attitude_sp =
    NwuToNed(control_inputs.attitude.quaternion).template cast<float>();
  attitude_setpoint_->update(attitude_sp, thrust_sp);
}

}  // namespace ros2_uav::modes

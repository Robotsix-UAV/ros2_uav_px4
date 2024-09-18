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

namespace ros2_uav::modes
{
template<DerivedFromRatesThrustMode ModeT>
RatesThrustMode<ModeT>::RatesThrustMode(
  const ModeBase::Settings & mode_settings,
  rclcpp::Node & node)
: ModeInterface<ModeT>(mode_settings, node)
{
  this->setSetpointUpdateRate(250.0);
  rates_setpoint_ = std::make_shared<px4_ros2::RatesSetpointType>(*this);
  this->addChildContainer(this->pipeline_.get());
  time_init_ = this->node_.now();
}

template<DerivedFromRatesThrustMode ModeT>
void RatesThrustMode<ModeT>::updateSetpoint([[maybe_unused]] float dt)
{
  this->odometryUpdate();
  auto elapsed_time = (this->node_.now()).nanoseconds();
  auto control_inputs = this->pipeline_->execute(std::chrono::nanoseconds(elapsed_time));
  const Eigen::Vector3f thrust_sp{0.0f, 0.0f, -static_cast<float>(control_inputs.thrust)};
  const Eigen::Vector3f rates_sp = NwuToNed(control_inputs.rates.vector).template cast<float>();
  rates_setpoint_->update(rates_sp, thrust_sp);
}

}  // namespace ros2_uav::modes

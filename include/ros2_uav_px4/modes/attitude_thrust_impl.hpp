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
#include "ros2_uav_px4/utils/tf2_eigen.hpp"

namespace ros2_uav::modes
{
using uav_ros2::utils::eigenNedToTf2Nwu;
using uav_ros2::utils::tf2FwuToEigenNed;

template<typename ModeT>
AttitudeThrustMode<ModeT>::AttitudeThrustMode(
  const ModeBase::Settings & mode_settings,
  rclcpp::Node & node)
: ModeInterface<ModeT>(mode_settings, node)
{
  this->setSetpointUpdateRate(250.0);
  attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
  vehicle_local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  vehicle_angular_velocity_ = std::make_shared<px4_ros2::OdometryAngularVelocity>(*this);
  vehicle_attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(*this);
  this->addRequiredParameter("px4.thrust_constant_coefficient", std::type_index(typeid(0.0)));
  this->addRequiredParameter("px4.thrust_linear_coefficient", std::type_index(typeid(0.0)));
  this->addRequiredParameter("px4.thrust_quadratic_coefficient", std::type_index(typeid(0.0)));
  this->addChildContainer(&(this->mode_));
  time_init_ = this->node_.now();
}

template<typename ModeT>
void AttitudeThrustMode<ModeT>::odometryUpdate()
{
  auto position = vehicle_local_position_->positionNed();
  auto velocity = vehicle_local_position_->velocityNed();
  auto attitude = vehicle_attitude_->attitude();
  auto angular_velocity = vehicle_angular_velocity_->angularVelocityFrd();
  this->mode_.setCurrentOdometry(
    eigenNedToTf2Nwu(position),
    eigenNedToTf2Nwu(attitude),
    eigenNedToTf2Nwu(velocity),
    eigenNedToTf2Nwu(angular_velocity));
}

template<typename ModeT>
void AttitudeThrustMode<ModeT>::updateSetpoint([[maybe_unused]] float dt)
{
  odometryUpdate();
  double elapsed_time = (this->node_.now() - time_init_).seconds();
  AttitudeThrust control_inputs = this->mode_.triggerMode(elapsed_time);
  // Publish the coordinates for debug
  auto coordinates = this->mode_.getCoordinates();
  this->publishCoordinates(coordinates);
  // Set the attitude setpoint
  // Conversion of the thrust
  double thrust_constant_coefficient, thrust_linear_coefficient, thrust_quadratic_coefficient;
  this->getParameter("px4.thrust_constant_coefficient", thrust_constant_coefficient);
  this->getParameter("px4.thrust_linear_coefficient", thrust_linear_coefficient);
  this->getParameter("px4.thrust_quadratic_coefficient", thrust_quadratic_coefficient);
  float thrust = control_inputs.thrust;
  float normalized_thrust = 0.0;
  // Find the normalized with thrust = c + l * t_n + q * t_n^2
  float discriminant = thrust_linear_coefficient * thrust_linear_coefficient -
    4 * thrust_quadratic_coefficient * (thrust_constant_coefficient - thrust);
  if (discriminant < 0) {
    RCLCPP_WARN(
      this->node_.get_logger(),
      "[Attitude Thrust Mode] Negative discriminant in thrust conversion");
  } else {
    normalized_thrust = (-thrust_linear_coefficient + std::sqrt(discriminant)) /
      (2 * thrust_quadratic_coefficient);
  }
  const Eigen::Vector3f thrust_sp{0.0f, 0.0f, static_cast<float>(-normalized_thrust)};
  const Eigen::Quaternionf attitude_sp = tf2FwuToEigenNed(control_inputs.orientation);
  attitude_setpoint_->update(attitude_sp, thrust_sp);
}

}  // namespace ros2_uav::modes

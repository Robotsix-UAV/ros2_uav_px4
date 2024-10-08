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

#include "ros2_uav_px4/model_identification/thrust_matcher_node.hpp"
#include "ros2_uav_px4/utils/type_conversions.hpp"

namespace ros2_uav::identification
{
ThrustMatcher::ThrustMatcher()
: Node("attitude_thrust_matcher"),
  LogTagHolder("AttitudeThrust Matcher"),
  model_matcher_(sampling_time_, {"limits.max_angle", "limits.min_z_acceleration",
      "model.vehicle_mass"})
{
  addChildContainer(&model_matcher_);
  // Set callback for the PX4 messages
  auto qos = rclcpp::QoS(1);
  qos.keep_last(1);
  qos.best_effort();
  qos.transient_local();
  actuators_suscriber_ = this->create_subscription<ActuatorMotors>(
    "fmu/out/actuator_motors", qos,
    std::bind(&ThrustMatcher::actuatorMotorsCallback, this, std::placeholders::_1));
  odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
    "fmu/out/vehicle_odometry", qos,
    std::bind(&ThrustMatcher::odometryCallback, this, std::placeholders::_1));
  control_mode_subscriber_ = this->create_subscription<VehicleControlMode>(
    "fmu/out/vehicle_control_mode", qos,
    std::bind(&ThrustMatcher::controlModeCallback, this, std::placeholders::_1));
  acceleration_subscriber_ = this->create_subscription<VehicleAcceleration>(
    "fmu/out/vehicle_acceleration", qos,
    std::bind(&ThrustMatcher::accelerationCallback, this, std::placeholders::_1));
}

void ThrustMatcher::actuatorMotorsCallback(const ActuatorMotors::SharedPtr actuator_motors)
{
  if (status_ == Status::COLLECTING) {
    uav_cpp::types::ThrustStamped thrust;
    thrust.timestamp = std::chrono::microseconds{actuator_motors->timestamp};
    thrust.value = (actuator_motors->control[0] + actuator_motors->control[1] +
      actuator_motors->control[2] + actuator_motors->control[3]) / 4.0;
    thrusts_.push_back(thrust);
    // Log actuator motors (for debug)
    actuator_timestamps_.push_back(thrust.timestamp);
    actuators_.push_back(
      {actuator_motors->control[0], actuator_motors->control[1],
        actuator_motors->control[2], actuator_motors->control[3]});
  }
}

void ThrustMatcher::odometryCallback(const VehicleOdometry::SharedPtr odometry)
{
  double altitude = -odometry->position[2];
  // Logic to trigger data collection
  if (altitude > trigger_altitude_ && status_ == Status::INIT) {
    if (trigger_counter_ >= trigger_validation_) {
      UAVCPP_INFO_TAG(this, "[Model Identification] Triggering data collection");
      status_ = Status::COLLECTING;
      trigger_counter_ = 0;
    } else {
      trigger_counter_++;
    }
  } else if (altitude < trigger_altitude_ && status_ == Status::COLLECTING) {
    if (trigger_counter_ >= trigger_validation_) {
      UAVCPP_INFO_TAG(this, "[Model Identification] Stopping data collection");
      status_ = Status::WAITING_DISARM;
    } else {
      trigger_counter_++;
    }
  } else {
    trigger_counter_ = 0;
  }

  // Collection of velocity data and orientation data
  if (status_ == Status::COLLECTING) {
    odometries_.push_back(uav_ros2::utils::convert(*odometry));
  }
}

void ThrustMatcher::accelerationCallback(
  const VehicleAcceleration::SharedPtr acceleration)
{
  if (status_ == Status::COLLECTING) {
    uav_cpp::types::AccelerationStamped acc;
    acc.timestamp = std::chrono::microseconds{acceleration->timestamp};
    acc.vector = Eigen::Vector3d(
      acceleration->xyz[0], -acceleration->xyz[1],
      -acceleration->xyz[2]);
    accelerations_.push_back(acc);
  }
}

void ThrustMatcher::controlModeCallback(const VehicleControlMode::SharedPtr control_mode)
{
  if (status_ == Status::WAITING_DISARM && control_mode->flag_armed == false) {
    UAVCPP_INFO_TAG(this, "[Model Identification] Disarmed, matching model");
    status_ = Status::MATCHING;

    // Resample the data
    // Check that the data is not empty
    if (thrusts_.empty()) {
      UAVCPP_ERROR_TAG(this, "[Model Identification] No thrust data");
      return;
    }
    if (odometries_.empty()) {
      UAVCPP_ERROR_TAG(this, "[Model Identification] No odometry data");
      return;
    }
    if (accelerations_.empty()) {
      UAVCPP_ERROR_TAG(this, "[Model Identification] No acceleration data");
      return;
    }
    for (const auto & thrust : thrusts_) {
      UAVCPP_DATA("identification_thrust_input", thrust);
    }
    for (const auto & odometry : odometries_) {
      UAVCPP_DATA("identification_odometry", odometry);
    }
    for (const auto & acceleration : accelerations_) {
      UAVCPP_DATA("identification_acceleration", acceleration);
    }
    std::vector<uav_cpp::types::AccelerationStamped> resampled_accelerations;
    auto start_time = std::max(
      {thrusts_.front().timestamp,
        odometries_.front().timestamp, accelerations_.front().timestamp});
    auto end_time = std::min(
      {thrusts_.back().timestamp,
        odometries_.back().timestamp, accelerations_.back().timestamp});
    auto success = uav_cpp::utils::resampleUavCppData(
      accelerations_, resampled_accelerations, sampling_time_,
      start_time, end_time);
    std::vector<uav_cpp::types::ThrustStamped> resampled_thrusts;
    std::vector<uav_cpp::types::OdometryStamped> resampled_odometries;
    success = uav_cpp::utils::resampleUavCppData(
      thrusts_, resampled_thrusts, sampling_time_, start_time, end_time);
    success &= uav_cpp::utils::resampleUavCppData(
      odometries_, resampled_odometries, sampling_time_, start_time, end_time);
    if (!success) {
      UAVCPP_ERROR_TAG(this, "Resampling failed");
      return;
    }

    // Add the outputs to the model matcher
    for (size_t i = 0; i < resampled_thrusts.size(); ++i) {
      uav_cpp::types::AttitudeThrustStamped output;
      output.timestamp = resampled_thrusts[i].timestamp;
      output.thrust = resampled_thrusts[i];
      output.attitude.quaternion = resampled_odometries[i].attitude.quaternion;
      model_matcher_.addOutput(output);
    }

    // Add the inputs to the model matcher
    for (size_t i = 0; i < resampled_accelerations.size(); ++i) {
      uav_cpp::types::AccelerationAttitudeStamped input;
      input.timestamp = resampled_accelerations[i].timestamp;
      auto acceleration = resampled_accelerations[i].vector;
      // Data from IMU is in the body frame, to convert to the world frame and remove gravity
      auto orientation = resampled_odometries[i].attitude.quaternion.normalized();
      acceleration = orientation * acceleration;
      acceleration.z() = acceleration.z() - 9.81;
      input.acceleration.vector = acceleration;
      input.attitude.quaternion = resampled_odometries[i].attitude.quaternion;
      model_matcher_.addInput(input);
    }

    // Add the odometries
    model_matcher_.addOdometries(resampled_odometries);

    // Collect the current computation to compare with the optimized model
    auto computed_inputs_default = model_matcher_.computeInputs();

    // Match the model
    std::vector<double> optimized_parameters;
    std::vector<uav_cpp::types::AccelerationAttitudeStamped> computed_inputs, resampled_inputs;
    std::vector<uav_cpp::types::AttitudeThrustStamped> resampled_outputs;
    model_matcher_.matchModel(
      optimized_parameters, computed_inputs, resampled_inputs,
      resampled_outputs);
    for (const auto & input : computed_inputs) {
      UAVCPP_DATA("identification_computed_input", input);
    }
    for (const auto & input : resampled_inputs) {
      UAVCPP_DATA("identification_resampled_input", input);
    }
    for (const auto & output : resampled_outputs) {
      UAVCPP_DATA("identification_resampled_output", output);
    }
    uav_cpp::logger::LogManager::getInstance().flushDataSinks();
  }
}
}  // namespace ros2_uav::identification

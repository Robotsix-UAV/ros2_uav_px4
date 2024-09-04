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

#include <uav_cpp/module_io/module_io_utils.hpp>
#include "ros2_uav_px4/model_identification/thrust_matcher_node.hpp"
#include "ros2_uav_px4/utils/uav_cpp_ros2_conversions.hpp"

namespace ros2_uav::identification
{
AttitudeThrustMatcher::AttitudeThrustMatcher()
: Node("attitude_thrust_matcher"),
  model_matcher_(1ms, {"limits.max_angle", "limits.min_z_acceleration", "model.vehicle_mass"})
{
  addChildContainer(&model_matcher_);
  // Set callback for the PX4 messages
  auto qos = rclcpp::QoS(1);
  qos.keep_last(1);
  qos.best_effort();
  qos.transient_local();
  actuators_suscriber_ = this->create_subscription<ActuatorMotors>(
    "fmu/out/actuator_motors", qos,
    std::bind(&AttitudeThrustMatcher::actuatorMotorsCallback, this, std::placeholders::_1));
  odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
    "fmu/out/vehicle_odometry", qos,
    std::bind(&AttitudeThrustMatcher::odometryCallback, this, std::placeholders::_1));
  control_mode_subscriber_ = this->create_subscription<VehicleControlMode>(
    "fmu/out/vehicle_control_mode", qos,
    std::bind(&AttitudeThrustMatcher::controlModeCallback, this, std::placeholders::_1));
  acceleration_subscriber_ = this->create_subscription<VehicleAcceleration>(
    "fmu/out/vehicle_acceleration", qos,
    std::bind(&AttitudeThrustMatcher::accelerationCallback, this, std::placeholders::_1));
}

void AttitudeThrustMatcher::actuatorMotorsCallback(const ActuatorMotors::SharedPtr actuator_motors)
{
  if (status_ == Status::COLLECTING) {
    uav_cpp::pipelines::Thrust thrust;
    thrust.timestamp = std::chrono::microseconds{actuator_motors->timestamp};
    thrust.thrust = (actuator_motors->control[0] + actuator_motors->control[1] +
      actuator_motors->control[2] + actuator_motors->control[3]) / 4.0;
    thrusts_.push_back(thrust);
    // Log actuator motors (for debug)
    actuator_timestamps_.push_back(thrust.timestamp);
    actuators_.push_back(
      {actuator_motors->control[0], actuator_motors->control[1],
        actuator_motors->control[2], actuator_motors->control[3]});
  }
}

void AttitudeThrustMatcher::odometryCallback(const VehicleOdometry::SharedPtr odometry)
{
  double altitude = -odometry->position[2];
  // Logic to trigger data collection
  if (altitude > trigger_altitude_ && status_ == Status::INIT) {
    if (trigger_counter_ >= trigger_validation_) {
      UAVCPP_INFO("[Model Identification] Triggering data collection");
      status_ = Status::COLLECTING;
      trigger_counter_ = 0;
    } else {
      trigger_counter_++;
    }
  } else if (altitude < trigger_altitude_ && status_ == Status::COLLECTING) {
    if (trigger_counter_ >= trigger_validation_) {
      UAVCPP_INFO("[Model Identification] Stopping data collection");
      status_ = Status::WAITING_DISARM;
    } else {
      trigger_counter_++;
    }
  } else {
    trigger_counter_ = 0;
  }

  // Collection of velocity data and orientation data
  if (status_ == Status::COLLECTING) {
    odometries_.push_back(uav_ros2::utils::convertFromPx4Msg(*odometry));
  }
}

void AttitudeThrustMatcher::accelerationCallback(
  const VehicleAcceleration::SharedPtr acceleration)
{
  if (status_ == Status::COLLECTING) {
    uav_cpp::pipelines::Acceleration acc;
    acc.timestamp = std::chrono::microseconds{acceleration->timestamp};
    acc.acceleration = tf2::Vector3(
      acceleration->xyz[0], -acceleration->xyz[1],
      -acceleration->xyz[2]);
    accelerations_.push_back(acc);
  }
}

void AttitudeThrustMatcher::controlModeCallback(const VehicleControlMode::SharedPtr control_mode)
{
  if (status_ == Status::WAITING_DISARM && control_mode->flag_armed == false) {
    UAVCPP_INFO("[Model Identification] Disarmed, matching model");
    status_ = Status::MATCHING;
    // Log for debug
    data_logger_.setStartTime(odometries_.front().timestamp);
    std::vector<std::chrono::nanoseconds> thrust_timestamps, altitude_timestamps;
    std::vector<std::vector<double>> thrusts, altitudes;
    for (const auto & thrust : thrusts_) {
      thrust_timestamps.push_back(thrust.timestamp);
      thrusts.push_back({thrust.thrust});
    }
    for (const auto & odometry : odometries_) {
      altitude_timestamps.push_back(odometry.timestamp);
      altitudes.push_back({-odometry.odometry.position.z()});
    }
    data_logger_.logToFile("thrust.csv", thrust_timestamps, {"thrust"}, thrusts);
    data_logger_.logToFile("altitude.csv", altitude_timestamps, {"altitude"}, altitudes);
    data_logger_.logToFile(
      "actuators.csv", actuator_timestamps_, {"motor0", "motor1", "motor2",
        "motor3"}, actuators_);
    // Resample the data
    // Check that the data is not empty
    if (thrusts_.empty()) {
      UAVCPP_ERROR("[Model Identification] No thrust data");
      return;
    }
    if (odometries_.empty()) {
      UAVCPP_ERROR("[Model Identification] No odometry data");
      return;
    }
    if (accelerations_.empty()) {
      UAVCPP_ERROR("[Model Identification] No acceleration data");
      return;
    }
    std::vector<uav_cpp::pipelines::Acceleration> resampled_accelerations;
    auto start_time = std::max(
      {thrusts_.front().timestamp,
        odometries_.front().timestamp, accelerations_.front().timestamp});
    auto end_time = std::min(
      {thrusts_.back().timestamp,
        odometries_.back().timestamp, accelerations_.back().timestamp});
    auto success = uav_cpp::utils::resampleModuleIO(
      accelerations_, resampled_accelerations, sampling_time_,
      start_time, end_time);
    std::vector<uav_cpp::pipelines::Thrust> resampled_thrusts;
    std::vector<uav_cpp::pipelines::Odometry> resampled_odometries;
    success = uav_cpp::utils::resampleModuleIO(
      thrusts_, resampled_thrusts, sampling_time_, start_time, end_time);
    success &= uav_cpp::utils::resampleModuleIO(
      odometries_, resampled_odometries, sampling_time_, start_time, end_time);
    if (!success) {
      UAVCPP_ERROR("[Model Identification] Resampling failed");
      return;
    }

    // Add the outputs to the model matcher
    for (size_t i = 0; i < resampled_thrusts.size(); ++i) {
      uav_cpp::pipelines::AttitudeThrust output;
      output.timestamp = resampled_thrusts[i].timestamp;
      output.thrust = resampled_thrusts[i].thrust;
      output.orientation = resampled_odometries[i].odometry.orientation;
      model_matcher_.addOutput(output);
    }

    // Add the inputs to the model matcher
    for (size_t i = 0; i < resampled_accelerations.size(); ++i) {
      uav_cpp::pipelines::AccelerationQuaternion input;
      input.timestamp = resampled_accelerations[i].timestamp;
      tf2::Vector3 acceleration = resampled_accelerations[i].acceleration;
      // Data from IMU is in the body frame, to convert to the world frame and remove gravity
      tf2::Quaternion orientation = resampled_odometries[i].odometry.orientation.normalized();
      acceleration = tf2::quatRotate(orientation, acceleration);
      acceleration.setZ(acceleration.z() - 9.81);
      input.acceleration = acceleration;
      input.orientation = resampled_odometries[i].odometry.orientation;
      model_matcher_.addInput(input);
    }

    // Add the odometries
    model_matcher_.addOdometries(resampled_odometries);

    // Collect the current computation to compare with the optimized model
    auto computed_inputs_default = model_matcher_.computeInputs();

    // Match the model
    std::vector<double> optimized_parameters;
    std::vector<uav_cpp::pipelines::AccelerationQuaternion> computed_inputs, resampled_inputs;
    std::vector<uav_cpp::models::AttitudeThrustScaler::OutputType> resampled_outputs;
    model_matcher_.matchModel(
      optimized_parameters, computed_inputs, resampled_inputs,
      resampled_outputs);
    std::vector<std::vector<double>> output_data;
    std::vector<std::chrono::nanoseconds> timestamps;
    for (size_t i = 0; i < computed_inputs.size(); ++i) {
      timestamps.push_back(computed_inputs[i].timestamp);
      output_data.push_back(
        {computed_inputs[i].acceleration.x(),
          computed_inputs[i].acceleration.y(), computed_inputs[i].acceleration.z(),
          computed_inputs[i].orientation.x(),
          computed_inputs[i].orientation.y(), computed_inputs[i].orientation.z(),
          computed_inputs[i].orientation.w(),
          resampled_inputs[i].acceleration.x(),
          resampled_inputs[i].acceleration.y(), resampled_inputs[i].acceleration.z(),
          resampled_inputs[i].orientation.x(),
          resampled_inputs[i].orientation.y(),
          resampled_inputs[i].orientation.z(), resampled_inputs[i].orientation.w(),
          resampled_outputs[i].thrust,
          computed_inputs_default[i].acceleration.x(),
          computed_inputs_default[i].acceleration.y(),
          computed_inputs_default[i].acceleration.z(),
          computed_inputs_default[i].orientation.x(),
          computed_inputs_default[i].orientation.y(),
          computed_inputs_default[i].orientation.z(),
          computed_inputs_default[i].orientation.w()});
    }
    data_logger_.logToFile(
      "identification.csv", timestamps, {"computed_acceleration_x", "computed_acceleration_y",
        "computed_acceleration_z", "computed_orientation_x", "computed_orientation_y",
        "computed_orientation_z", "computed_orientation_w",
        "resampled_acceleration_x", "resampled_acceleration_y", "resampled_acceleration_z",
        "resampled_orientation_x", "resampled_orientation_y", "resampled_orientation_z",
        "resampled_orientation_w", "resampled_thrust",
        "default_acceleration_x", "default_acceleration_y", "default_acceleration_z",
        "default_orientation_x", "default_orientation_y", "default_orientation_z",
        "default_orientation_w"},
      output_data);
  }
}
}  // namespace ros2_uav::identification

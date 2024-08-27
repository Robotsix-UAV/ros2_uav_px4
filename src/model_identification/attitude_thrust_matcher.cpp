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
#include "ros2_uav_px4/model_identification/attitude_thrust_matcher.hpp"
#include "ros2_uav_px4/utils/uav_cpp_ros2_conversions.hpp"

namespace ros2_uav::identification
{
AttitudeThrustMatcher::AttitudeThrustMatcher()
: Node("attitude_thrust_matcher"),
  derivative_filter_(30,
    1 / std::chrono::duration_cast<std::chrono::duration<double>>(sampling_time_).count()),
  model_matcher_(1ms, {"limits.max_angle", "limits.min_z_acceleration", "model.vehicle_mass"})
{
  addChildContainer(&model_matcher_);
  // Set callback for the PX4 messages
  auto qos = rclcpp::QoS(1);
  qos.keep_last(1);
  qos.best_effort();
  qos.transient_local();
  attitude_setpoint_subscriber_ = this->create_subscription<VehicleAttitudeSetpoint>(
    "fmu/in/vehicle_attitude_setpoint", 1,
    std::bind(&AttitudeThrustMatcher::attitudeSetpointCallback, this, std::placeholders::_1));
  odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
    "fmu/out/vehicle_odometry", qos,
    std::bind(&AttitudeThrustMatcher::odometryCallback, this, std::placeholders::_1));
  control_mode_subscriber_ = this->create_subscription<VehicleControlMode>(
    "fmu/out/vehicle_control_mode", qos,
    std::bind(&AttitudeThrustMatcher::controlModeCallback, this, std::placeholders::_1));
}

void AttitudeThrustMatcher::attitudeSetpointCallback(
  const VehicleAttitudeSetpoint::SharedPtr attitude_setpoint)
{
  if (status_ == Status::COLLECTING) {
    auto att_thrust = uav_ros2::utils::convertFromPx4Msg(*attitude_setpoint);
    model_matcher_.addOutput(att_thrust);
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
    VelocityQuaternion vel_quat;
    vel_quat.timestamp = std::chrono::microseconds{odometry->timestamp};
    vel_quat.velocity.setX(odometry->velocity[0]);
    vel_quat.velocity.setY(-odometry->velocity[1]);
    vel_quat.velocity.setZ(-odometry->velocity[2]);
    vel_quat.orientation.setX(odometry->q[1]);
    vel_quat.orientation.setY(-odometry->q[2]);
    vel_quat.orientation.setZ(-odometry->q[3]);
    vel_quat.orientation.setW(odometry->q[0]);
    velocity_data_.push_back(vel_quat);
  }
}

void AttitudeThrustMatcher::controlModeCallback(const VehicleControlMode::SharedPtr control_mode)
{
  if (status_ == Status::WAITING_DISARM && control_mode->flag_armed == false) {
    UAVCPP_INFO("[Model Identification] Disarmed, matching model");
    status_ = Status::MATCHING;
    // Resample the data
    std::vector<VelocityQuaternion> resampled_data;
    auto success = uav_cpp::utils::resampleModuleIO(
      velocity_data_, resampled_data, sampling_time_,
      velocity_data_.front().timestamp, velocity_data_.back().timestamp);
    if (!success) {
      UAVCPP_ERROR("[Model Identification] Resampling failed");
      return;
    }
    // Derivate and filter the velocity
    std::vector<std::chrono::nanoseconds> timestamps;
    for (auto & vel_quat : resampled_data) {
      derivative_filter_.addData(vel_quat.velocity);
      timestamps.push_back(vel_quat.timestamp);
    }
    std::vector<tf2::Vector3> derivatives, derivatives_filtered;
    derivative_filter_.process(derivatives, derivatives_filtered);
    // Log the data
    std::vector<std::string> names =
    {"acceleration_x", "acceleration_y", "acceleration_z", "filtered_acceleration_x",
      "filtered_acceleration_y", "filtered_acceleration_z"};
    std::vector<std::vector<double>> data;
    for (size_t i = 0; i < derivatives.size(); ++i) {
      data.push_back(
        {derivatives[i].x(), derivatives[i].y(),
          derivatives[i].z(), derivatives_filtered[i].x(),
          derivatives_filtered[i].y(), derivatives_filtered[i].z()});
    }

    data_logger_.logToFile("acceleration.csv", timestamps, names, data);
    UAVCPP_INFO("[Model Identification] Data resampled and filtered");

    // Match the model
    std::vector<uav_cpp::pipelines::AccelerationQuaternion> resampled_input;
    for (size_t i = 0; i < resampled_data.size(); ++i) {
      uav_cpp::pipelines::AccelerationQuaternion acc_quat;
      acc_quat.timestamp = resampled_data[i].timestamp;
      acc_quat.acceleration = derivatives_filtered[i];
      acc_quat.orientation = resampled_data[i].orientation;
      resampled_input.push_back(acc_quat);
    }
    // Clip some samples at the beginning and at the end (not reliable)
    int clip_samples = 20;
    resampled_input.erase(resampled_input.begin(), resampled_input.begin() + clip_samples);
    resampled_input.erase(resampled_input.end() - clip_samples, resampled_input.end());
    model_matcher_.addInputs(resampled_input);
    std::vector<double> optimized_parameters;
    std::vector<uav_cpp::pipelines::AccelerationQuaternion> computed_inputs, resampled_inputs;
    std::vector<uav_cpp::models::AttitudeThrustScaler::OutputType> resampled_outputs;
    model_matcher_.matchModel(
      optimized_parameters, computed_inputs, resampled_inputs,
      resampled_outputs);
    std::vector<std::vector<double>> output_data;
    timestamps.clear();
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
          resampled_outputs[i].thrust});
    }
    data_logger_.logToFile(
      "output.csv", timestamps, {"computed_acceleration_x", "computed_acceleration_y",
        "computed_acceleration_z", "computed_orientation_x", "computed_orientation_y",
        "computed_orientation_z", "computed_orientation_w",
        "resampled_acceleration_x", "resampled_acceleration_y", "resampled_acceleration_z",
        "resampled_orientation_x", "resampled_orientation_y", "resampled_orientation_z",
        "resampled_orientation_w", "resampled_thrust"}, output_data);
  }
}
}  // namespace ros2_uav::identification

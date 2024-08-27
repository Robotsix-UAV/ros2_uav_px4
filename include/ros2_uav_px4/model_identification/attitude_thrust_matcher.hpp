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

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <uav_cpp/model_identification/model_matcher.hpp>
#include <uav_cpp/vehicle_models/quadrotor.hpp>
#include <uav_cpp/vehicle_models/thrust_scaler.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "ros2_uav_px4/utils/data_logger.hpp"
#include "ros2_uav_px4/utils/derivative_filter.hpp"

namespace ros2_uav::identification
{
using uav_cpp::identification::ModelMatcher;
using uav_cpp::models::QuadrotorModel;
using uav_cpp::models::AttitudeThrustScaler;
using uav_cpp::pipelines::VelocityQuaternion;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleControlMode;
using std::chrono_literals::operator""ms;

/**
 * @brief Class for matching parameters governing the attitude and thrust control of a UAV.
 */
class AttitudeThrustMatcher : public rclcpp::Node, public uav_cpp::parameters::ParamContainer
{
public:
  enum class Status
  {
    INIT,
    COLLECTING,
    WAITING_DISARM,
    MATCHING
  };

  AttitudeThrustMatcher();

  /**
   * @brief Callback for the attitude setpoint.
   * @param attitude_setpoint Attitude setpoint message.
   */
  void attitudeSetpointCallback(const VehicleAttitudeSetpoint::SharedPtr attitude_setpoint);

  /**
   * @brief Callback for the odometry.
   * @param odometry Odometry message.
   */
  void odometryCallback(const VehicleOdometry::SharedPtr odometry);

  /**
   * @brief Callback for the control mode.
   * @param control_mode Control mode message.
   */
  void controlModeCallback(const VehicleControlMode::SharedPtr control_mode);

private:
  Status status_ = Status::INIT;   /**< Status of the attitude thrust matcher. */
  std::chrono::milliseconds sampling_time_ = 1ms;   /**< Sampling time. */
  uav_ros2::utils::DataLogger data_logger_;   /**< Data logger. */
  double trigger_altitude_ = 2.0;   /**< Altitude at which the data collection is triggered. */
  uint8_t trigger_validation_ = 30;   /**< Number of samples to validate the trigger. */
  uint8_t trigger_counter_ = 0;   /**< Counter for the trigger validation. */
  rclcpp::Subscription<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_subscriber_;
  /**< Subscriber for the attitude setpoint. */
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscriber_;
  /**< Subscriber for the odometry. */
  rclcpp::Subscription<VehicleControlMode>::SharedPtr control_mode_subscriber_;
  /**< Subscriber for the control mode. */
  std::vector<VelocityQuaternion> velocity_data_;   /**< Vector of velocity data. */
  uav_ros2::utils::DerivativeFilter3D derivative_filter_;
  /**< Derivative filter for the velocity. */
  uav_cpp::identification::ModelMatcher<QuadrotorModel, AttitudeThrustScaler> model_matcher_;
  /**< Model matcher for the quadrotor and the attitude thrust scaler. */
};
}  // namespace ros2_uav::identification

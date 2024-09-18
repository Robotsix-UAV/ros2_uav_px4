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

#include <ros2_uav_interfaces/msg/pose_heading.hpp>
#include <uav_cpp/types/enums.hpp>
#include <uav_cpp/types/timestamped_types.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace uav_ros2::utils
{
/**
 * @brief Converts a frame_id to a uavcpp::types::FrameId.
 * @param frame_id The frame_id to convert.
 * @return The converted FrameId type.
 */
uav_cpp::types::Frame convertFrameId(const std::string & frame_id)
{
  if (frame_id.find("odom") != std::string::npos) {
    return uav_cpp::types::Frame::ODOM;
  } else if (frame_id.find("base_link") != std::string::npos) {
    return uav_cpp::types::Frame::BASE_LINK;
  } else {
    return uav_cpp::types::Frame::UNKNOWN;
  }
}

/**
 * @brief Converts a ros2_uav_interfaces/PoseHeading message to a uavcpp::types::PoseHeadingStamped.
 * @param msg The PoseHeading message to convert.
 * @return The converted PoseHeadingStamped type.
 */
uav_cpp::types::PoseHeadingStamped convertFromRosMsg(
  const ros2_uav_interfaces::msg::PoseHeading & msg)
{
  uav_cpp::types::PoseHeadingStamped setpoint;
  setpoint.timestamp =
    std::chrono::nanoseconds{static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000 +
    static_cast<uint64_t>(msg.header.stamp.nanosec)};
  setpoint.frame_id = convertFrameId(msg.header.frame_id);
  setpoint.position = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  setpoint.velocity = Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
  setpoint.heading = msg.heading;
  return setpoint;
}

/**
 * @brief Converts a px4_msgs/VehicleAttitudeSetpoint message to a uavcpp::types::AttitudeThrustStamped.
 * @param msg The VehicleAttitudeSetpoint message to convert.
 * @return The converted AttitudeThrust type.
 */
uav_cpp::types::AttitudeThrustStamped convertFromPx4Msg(
  const px4_msgs::msg::VehicleAttitudeSetpoint & msg)
{
  uav_cpp::types::AttitudeThrustStamped att_thrust;
  att_thrust.timestamp = std::chrono::microseconds{msg.timestamp};
  att_thrust.attitude = Eigen::Quaterniond(msg.q_d[0], msg.q_d[1], -msg.q_d[2], -msg.q_d[3]);
  att_thrust.thrust = -msg.thrust_body[2];
  return att_thrust;
}

/**
 * @brief Converts a px4_msgs/VehicleOdometry message to a uavcpp::types::OdometryStamped.
 * @param msg The VehicleOdometry message to convert.
 * @return The converted Odometry type.
 */
uav_cpp::types::OdometryStamped convertFromPx4Msg(const px4_msgs::msg::VehicleOdometry & msg)
{
  uav_cpp::types::OdometryStamped odometry;
  odometry.timestamp = std::chrono::microseconds{msg.timestamp};
  odometry.position = Eigen::Vector3d(msg.position[0], -msg.position[1], -msg.position[2]);
  odometry.velocity = Eigen::Vector3d(msg.velocity[0], -msg.velocity[1], -msg.velocity[2]);
  odometry.attitude = Eigen::Quaterniond(msg.q[0], msg.q[1], -msg.q[2], -msg.q[3]);
  odometry.angular_velocity = Eigen::Vector3d(
    msg.angular_velocity[0],
    -msg.angular_velocity[1],
    -msg.angular_velocity[2]);
  return odometry;
}

/**
 * @brief Converts a px4_msgs/VehicleThrustSetpoint message to a uavcpp::types::ThrustStamped.
 * @param msg The VehicleThrustSetpoint message to convert.
 * @return The converted Thrust type.
 */
uav_cpp::types::ThrustStamped convertFromPx4Msg(const px4_msgs::msg::VehicleThrustSetpoint & msg)
{
  uav_cpp::types::ThrustStamped thrust;
  thrust.timestamp = std::chrono::microseconds{msg.timestamp};
  thrust.value = -msg.xyz[2];
  return thrust;
}
}  // namespace uav_ros2::utils

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

#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <ros2_uav_interfaces/msg/disturbance.hpp>
#include <ros2_uav_interfaces/msg/pose_heading.hpp>
#include <ros2_uav_interfaces/msg/waypoint_list.hpp>
#include <string>
#include <uav_cpp/types/enums.hpp>
#include <uav_cpp/types/timestamped_types.hpp>

#include "arrc_interfaces/msg/uav_pose.hpp"

namespace ros2_uav::utils {
/**
 * @brief Converts a frame_id to a uavcpp::types::FrameId.
 * @param frame_id The frame_id to convert.
 * @return The converted FrameId type.
 */
inline uav_cpp::types::Frame convert(const std::string& frame_id) {
  if (frame_id.find("odom") != std::string::npos || frame_id.find("gps_origin") != std::string::npos) {
    return uav_cpp::types::Frame::ODOM;
  } else if (frame_id.find("base_link") != std::string::npos || frame_id.find("fcu") != std::string::npos) {
    return uav_cpp::types::Frame::BASE_LINK;
  } else {
    return uav_cpp::types::Frame::UNKNOWN;
  }
}

/**
 * @brief Converts a uavcpp::types::FrameId to a frame_id.
 * @param frame_id The FrameId to convert.
 * @return The converted frame_id.
 */
inline std::string convert(const uav_cpp::types::Frame& frame_id) {
  switch (frame_id) {
    case uav_cpp::types::Frame::ODOM:
      return "odom";
    case uav_cpp::types::Frame::BASE_LINK:
      return "base_link";
    default:
      return "unknown";
  }
}

/**
 * @brief Converts a ros2_uav_interfaces/PoseHeading message to a
 * uavcpp::types::PoseHeadingStamped.
 * @param msg The PoseHeading message to convert.
 * @return The converted PoseHeadingStamped type.
 */
inline uav_cpp::types::PoseHeadingStamped convert(
    const ros2_uav_interfaces::msg::PoseHeading& msg) {
  uav_cpp::types::PoseHeadingStamped setpoint;
  setpoint.timestamp = std::chrono::nanoseconds{
      static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000 +
      static_cast<uint64_t>(msg.header.stamp.nanosec)};
  setpoint.frame_id = convert(msg.header.frame_id);
  setpoint.position =
      Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  setpoint.velocity =
      Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
  setpoint.heading = msg.heading;
  return setpoint;
}

inline uav_cpp::types::PoseHeadingStamped convert(
    const arrc_interfaces::msg::UavPose& msg) {
  uav_cpp::types::PoseHeadingStamped setpoint;
  setpoint.timestamp = std::chrono::nanoseconds{
      static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000 +
      static_cast<uint64_t>(msg.header.stamp.nanosec)};
  setpoint.frame_id = convert(msg.header.frame_id);
  setpoint.position = Eigen::Vector3d(msg.pose.x, msg.pose.y, msg.pose.z);
  setpoint.heading = msg.pose.yaw;
  return setpoint;
}

/**
 * @brief Converts a ros2_uav_interfaces/Disturbance message to a
 * uavcpp::types::DisturbanceCoefficientsStamped.
 * @param msg The Disturbance message to convert.
 * @return The converted DisturbanceCoefficientsStamped type.
 */
inline uav_cpp::types::DisturbanceCoefficientsStamped convert(
    const ros2_uav_interfaces::msg::Disturbance& msg) {
  uav_cpp::types::DisturbanceCoefficientsStamped disturbance;
  disturbance.timestamp = std::chrono::nanoseconds{
      static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000 +
      static_cast<uint64_t>(msg.header.stamp.nanosec)};
  disturbance.frame_id = convert(msg.header.frame_id);
  disturbance.constant =
      Eigen::Vector3d(msg.constant.x, msg.constant.y, msg.constant.z);
  disturbance.proportional = Eigen::Vector3d(
      msg.proportional.x, msg.proportional.y, msg.proportional.z);
  return disturbance;
}

/**
 * @brief Converts a uavcpp::types::DisturbanceCoefficientsStamped message to a
 * ros2_uav_interfaces/Disturbance.
 * @param disturbance The DisturbanceCoefficientsStamped message to convert.
 */
inline ros2_uav_interfaces::msg::Disturbance convert(
    const uav_cpp::types::DisturbanceCoefficientsStamped& disturbance) {
  ros2_uav_interfaces::msg::Disturbance msg;
  msg.header.stamp.sec = disturbance.timestamp.count() / 1000000000;
  msg.header.stamp.nanosec = disturbance.timestamp.count() % 1000000000;
  msg.header.frame_id = convert(disturbance.frame_id);
  msg.constant.x = disturbance.constant.vector.x();
  msg.constant.y = disturbance.constant.vector.y();
  msg.constant.z = disturbance.constant.vector.z();
  msg.proportional.x = disturbance.proportional.vector.x();
  msg.proportional.y = disturbance.proportional.vector.y();
  msg.proportional.z = disturbance.proportional.vector.z();
  return msg;
}

/**
 * @brief Converts a px4_msgs/VehicleAttitudeSetpoint message to a
 * uavcpp::types::AttitudeThrustStamped.
 * @param msg The VehicleAttitudeSetpoint message to convert.
 * @return The converted AttitudeThrust type.
 */
inline uav_cpp::types::AttitudeThrustStamped convert(
    const px4_msgs::msg::VehicleAttitudeSetpoint& msg) {
  uav_cpp::types::AttitudeThrustStamped att_thrust;
  att_thrust.timestamp = std::chrono::microseconds{msg.timestamp};
  att_thrust.attitude =
      Eigen::Quaterniond(msg.q_d[0], msg.q_d[1], -msg.q_d[2], -msg.q_d[3]);
  att_thrust.thrust = -msg.thrust_body[2];
  return att_thrust;
}

/**
 * @brief Converts a px4_msgs/VehicleOdometry message to a
 * uavcpp::types::OdometryStamped.
 * @param msg The VehicleOdometry message to convert.
 * @return The converted Odometry type.
 */
inline uav_cpp::types::OdometryStamped convert(
    const px4_msgs::msg::VehicleOdometry& msg) {
  uav_cpp::types::OdometryStamped odometry;
  odometry.timestamp = std::chrono::microseconds{msg.timestamp};
  odometry.position =
      Eigen::Vector3d(msg.position[0], -msg.position[1], -msg.position[2]);
  odometry.velocity =
      Eigen::Vector3d(msg.velocity[0], -msg.velocity[1], -msg.velocity[2]);
  odometry.attitude =
      Eigen::Quaterniond(msg.q[0], msg.q[1], -msg.q[2], -msg.q[3]);
  odometry.angular_velocity =
      Eigen::Vector3d(msg.angular_velocity[0], -msg.angular_velocity[1],
                      -msg.angular_velocity[2]);
  return odometry;
}

/**
 * @brief Converts a px4_msgs/VehicleThrustSetpoint message to a
 * uavcpp::types::ThrustStamped.
 * @param msg The VehicleThrustSetpoint message to convert.
 * @return The converted Thrust type.
 */
inline uav_cpp::types::ThrustStamped convert(
    const px4_msgs::msg::VehicleThrustSetpoint& msg) {
  uav_cpp::types::ThrustStamped thrust;
  thrust.timestamp = std::chrono::microseconds{msg.timestamp};
  thrust.value = -msg.xyz[2];
  return thrust;
}

inline uav_cpp::types::PoseSpeedVectorStamped convert(
    const ros2_uav_interfaces::msg::WaypointList& msg) {
  uav_cpp::types::PoseSpeedVectorStamped pose_speed_vector;
  pose_speed_vector.timestamp = std::chrono::nanoseconds{
      static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000 +
      static_cast<uint64_t>(msg.header.stamp.nanosec)};
  pose_speed_vector.frame_id = convert(msg.header.frame_id);
  for (const auto& waypoint : msg.waypoints) {
    uav_cpp::types::PoseSpeed pose_speed;
    pose_speed.position = Eigen::Vector3d(
        waypoint.position.x, waypoint.position.y, waypoint.position.z);
    pose_speed.speed = waypoint.speed;
    pose_speed_vector.waypoints.push_back(pose_speed);
  }
  return pose_speed_vector;
}
}  // namespace ros2_uav::utils

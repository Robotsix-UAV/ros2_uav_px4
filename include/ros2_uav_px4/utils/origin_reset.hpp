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

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"


namespace ros2_uav::utils
{
/**
 * @brief Convenient class to reset the origin of the PX4 UAV.
 */
class OriginReset
{
public:
  /**
   * @brief Constructor
   * @param node Reference node for the pub/sub
   * @param target_system_id The UAV system id
   */
  OriginReset(
    rclcpp::Node & node,
    int target_system_id)
  : target_system_id_(target_system_id),
    node_(node)
  {
    vehicle_command_pub = node.create_publisher<px4_msgs::msg::VehicleCommand>(
      "fmu/in/vehicle_command", 1);
    vehicle_global_position_sub = node.create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "fmu/out/vehicle_global_position", rclcpp::QoS(1).best_effort(),
      [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
        current_lat = msg->lat;
        current_lon = msg->lon;
        current_alt = msg->alt;
        global_position_received = true;
      });
  }

  /**
   * @brief Reset the origin of the UAV
   * @return True if the origin was reset, false otherwise
   */
  bool resetOrigin()
  {
    if (global_position_received) {
      px4_msgs::msg::VehicleCommand msg;
      msg.timestamp = round(node_.now().nanoseconds() / 1000.0);
      msg.target_system = target_system_id_;
      msg.target_component = 1;
      msg.source_system = 255;
      msg.from_external = true;
      msg.param5 = current_lat;
      msg.param6 = current_lon;
      msg.param7 = current_alt;
      msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
      vehicle_command_pub->publish(msg);
      UAVCPP_INFO("Global position origin reset");
      return true;
    }
    return false;
  }

  sensor_msgs::msg::NavSatFix getOrigin()
  {
    sensor_msgs::msg::NavSatFix origin;
    origin.header.stamp = node_.now();
    origin.latitude = current_lat;
    origin.longitude = current_lon;
    origin.altitude = current_alt;
    return origin;
  }


private:
  int target_system_id_;  ///< The UAV system id
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
  ///< Publisher for the vehicle command
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub;
  ///< Subscription for the vehicle global position
  rclcpp::Node & node_;  ///< Reference node for the pub/sub
  bool global_position_received = false;  ///< Flag to know if the global position was received
  float current_lat = 0;  ///< Current latitude
  float current_lon = 0;  ///< Current longitude
  float current_alt = 0;  ///< Current altitude
};
}  // namespace ros2_uav::utils

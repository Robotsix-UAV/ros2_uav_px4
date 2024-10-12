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

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

/** Base class for communication with a PX4 FCU.
 * Generated automatically from the dd_topics.yaml file.
 */
class Px4CommBase
{
public:
  /**
   * Constructor
   *
   * @param node A pointer to the ROS2 node.
   */
  explicit Px4CommBase(rclcpp::Node * node)
  : node_(node)
  {
    // Setting qos
    auto qos = rclcpp::QoS(1);
    qos.keep_last(1);
    qos.best_effort();
    qos.transient_local();


    offboard_control_mode_pub = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "fmu/in/offboard_control_mode", qos);
    vehicle_attitude_setpoint_pub = node_->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "fmu/in/vehicle_attitude_setpoint", qos);
    vehicle_rates_setpoint_pub = node_->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "fmu/in/vehicle_rates_setpoint", qos);
    vehicle_command_pub = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
      "fmu/in/vehicle_command", qos);


    vehicle_odometry_sub = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry", qos, [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        onVehicleOdometry(msg);
      });
    vehicle_status_sub = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
      "fmu/out/vehicle_status", qos, [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        onVehicleStatus(msg);
      });
    vehicle_control_mode_sub = node_->create_subscription<px4_msgs::msg::VehicleControlMode>(
      "fmu/out/vehicle_control_mode", qos,
      [this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {onVehicleControlMode(msg);});
  }

protected:
  rclcpp::Node * node_;  /**< The ROS2 node */

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
    vehicle_odometry_sub;
  /**< Subscriber for px4_msgs::msg::VehicleOdometry */
  /**
   * px4_msgs::msg::VehicleOdometry callback
   * @param msg Message from PX4
   */
  virtual void onVehicleOdometry(
    [[maybe_unused]] const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {}
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub;
  /**< Subscriber for px4_msgs::msg::VehicleStatus */
  /**
   * px4_msgs::msg::VehicleStatus callback
   * @param msg Message from PX4
   */
  virtual void onVehicleStatus([[maybe_unused]] const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
  }
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr
    vehicle_control_mode_sub;
  /**< Subscriber for px4_msgs::msg::VehicleControlMode */
  /**
   * px4_msgs::msg::VehicleControlMode callback
   * @param msg Message from PX4
   */
  virtual void onVehicleControlMode(
    [[maybe_unused]] const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {}


  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub;
  /**< Publisher for px4_msgs::msg::OffboardControlMode */
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    vehicle_attitude_setpoint_pub;
  /**< Publisher for px4_msgs::msg::VehicleAttitudeSetpoint */
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
    vehicle_rates_setpoint_pub;
  /**< Publisher for px4_msgs::msg::VehicleRatesSetpoint */
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub;
  /**< Publisher for px4_msgs::msg::VehicleCommand */};

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

#include <memory>
#include <mutex>
#include <thread>

#include <uav_cpp/types/timestamped_types.hpp>
#include <uav_cpp/interfaces/fcu_interface.hpp>
#include "ros2_uav_px4/px4_interface/px4_comm_base.hpp"

namespace ros2_uav
{
using uav_cpp::types::AttitudeThrustStamped;
using uav_cpp::types::RatesThrustStamped;

/**
 * Class for handling PX4 communication using ROS
 */
class Px4Comm : public Px4CommBase
{
public:
  /**
   * Constructor
   * @param node parent ROS2 node
   */
  explicit Px4Comm(rclcpp::Node * node);

  /**
   * Destructor
   */
  ~Px4Comm();

  void setArm(bool arm);
  void setOffboard(bool offboard);
  void land();
  void takeoff();
  void landHome();
  void setAttitudeThrust(const AttitudeThrustStamped & setpoint);
  void setRatesThrust(const RatesThrustStamped & setpoint);
  void pingOffboard();
  bool isConnected() {return connected_;}
  int getTargetSystem() {return target_system_;}
  void setFcuInterface(std::shared_ptr<uav_cpp::fcu_interface::FcuInterface> fcu_interface)
  {
    fcu_interface_ = fcu_interface;
  }

private:
  int target_system_; /**< Target system ID */
  bool connected_{false}; /**< Connection status */
  std::shared_ptr<uav_cpp::fcu_interface::FcuInterface> fcu_interface_;
  /**< Pointer to the FCU interface */

  int64_t last_status_received_{0};   /**< Last time the status was received */
  std::mutex mtx_;                    /**< Mutex for synchronization */
  std::unique_ptr<std::jthread> check_connection_thread_;
  /**< Thread for checking connection status */

  /**
   * @copydoc Px4CommBase::onVehicleControlMode
   */
  void onVehicleControlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) override;

  /**
   * @copydoc Px4CommBase::onVehicleOdometry
   */
  void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) override;

  /**
   * @copydoc Px4CommBase::onVehicleStatus
   */
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) override;

  /**
   * Format the timestamp for PX4 messages
   * @return Timestamp in microseconds
   */
  int64_t Px4TimestampNow();

  /**
   * Set default command values
   * @param msg Vehicle command message to send
   */
  void DefaultCommand(px4_msgs::msg::VehicleCommand & msg);

  /**
   * Check if a status message has been received for disconnection check
   */
  void checkStatusReceived();
};

}  // namespace ros2_uav

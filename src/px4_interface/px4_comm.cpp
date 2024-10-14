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

#include <chrono>
#include <cmath>
#include <limits>
#include <uav_cpp/manager/core_manager.hpp>
#include <uav_cpp/types/timestamped_types.hpp>
#include "ros2_uav_px4/px4_interface/px4_comm.hpp"
#include "ros2_uav_px4/utils/type_conversions.hpp"

namespace ros2_uav
{

Px4Comm::Px4Comm(rclcpp::Node * node)
: Px4CommBase(node)
{
  check_connection_thread_ = std::make_unique<std::jthread>(
    [this](std::stop_token stop_token) {
      while (!stop_token.stop_requested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        checkStatusReceived();
      }
    });
}

Px4Comm::~Px4Comm()
{
  if (check_connection_thread_) {
    check_connection_thread_->request_stop();
    check_connection_thread_->join();
  }
}

void Px4Comm::setArm(bool arm)
{
  px4_msgs::msg::VehicleCommand msg;
  DefaultCommand(msg);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg.param1 = (arm) ? 1.0 : 0.0;
  vehicle_command_pub->publish(msg);
}

void Px4Comm::setOffboard(bool offboard)
{
  px4_msgs::msg::VehicleCommand msg;
  DefaultCommand(msg);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  msg.param1 = (offboard) ? 1.0 : 0.0;
  msg.param2 = 6.0;
  vehicle_command_pub->publish(msg);
}

void Px4Comm::land()
{
  px4_msgs::msg::VehicleCommand msg;
  DefaultCommand(msg);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  msg.param1 = 1.0;
  vehicle_command_pub->publish(msg);
}

void Px4Comm::landHome()
{
  px4_msgs::msg::VehicleCommand msg;
  DefaultCommand(msg);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_LAND_START;
  vehicle_command_pub->publish(msg);
}

void Px4Comm::takeoff()
{
  px4_msgs::msg::VehicleCommand msg;
  DefaultCommand(msg);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  msg.param1 = 1.0;
  vehicle_command_pub->publish(msg);
}

void Px4Comm::setAttitudeThrust(const AttitudeThrustStamped & input)
{
  px4_msgs::msg::VehicleAttitudeSetpoint msg_set_point;
  msg_set_point.timestamp = Px4TimestampNow();
  float scaled_thrust;
  scaled_thrust = std::min(std::max(input.thrust.value, 0.0), 1.0);
  // Conversion from FLT to FRD representation
  msg_set_point.q_d = {
    static_cast<float>(input.attitude.quaternion.w()),
    static_cast<float>(input.attitude.quaternion.x()),
    -static_cast<float>(input.attitude.quaternion.y()),
    -static_cast<float>(input.attitude.quaternion.z())};
  // select always the quaternion with w > 0
  if (msg_set_point.q_d[0] < 0) {
    msg_set_point.q_d =
    {-msg_set_point.q_d[0], -msg_set_point.q_d[1], -msg_set_point.q_d[2],
      -msg_set_point.q_d[3]};
  }
  msg_set_point.thrust_body = {0.0, 0.0, -scaled_thrust};
  msg_set_point.yaw_sp_move_rate = NAN;

  msg_set_point.roll_body = NAN;
  msg_set_point.pitch_body = NAN;
  msg_set_point.yaw_body = NAN;

  px4_msgs::msg::OffboardControlMode msg_offboard;
  msg_offboard.timestamp = Px4TimestampNow();
  msg_offboard.attitude = true;

  offboard_control_mode_pub->publish(msg_offboard);
  usleep(1);
  vehicle_attitude_setpoint_pub->publish(msg_set_point);
  last_ping_offboard_ = node_->now().nanoseconds();
}

void Px4Comm::setRatesThrust(const RatesThrustStamped & input)
{
  px4_msgs::msg::VehicleRatesSetpoint msg_set_point;
  msg_set_point.timestamp = Px4TimestampNow();
  msg_set_point.roll = input.rates.vector.x();
  msg_set_point.pitch = -input.rates.vector.y();
  msg_set_point.yaw = -input.rates.vector.z();
  msg_set_point.thrust_body = {0, 0, -static_cast<float>(input.thrust.value)};

  px4_msgs::msg::OffboardControlMode msg_offboard;
  msg_offboard.timestamp = Px4TimestampNow();
  msg_offboard.body_rate = true;

  offboard_control_mode_pub->publish(msg_offboard);
  usleep(1);
  vehicle_rates_setpoint_pub->publish(msg_set_point);
  last_ping_offboard_ = node_->now().nanoseconds();
}


void Px4Comm::pingOffboard()
{
  px4_msgs::msg::OffboardControlMode msg_offboard;
  msg_offboard.timestamp = Px4TimestampNow();
  msg_offboard.attitude = true;
  offboard_control_mode_pub->publish(msg_offboard);
  last_ping_offboard_ = node_->now().nanoseconds();
}

int64_t Px4Comm::Px4TimestampNow()
{
  return round(node_->now().nanoseconds() / 1000.0);
}

void Px4Comm::DefaultCommand(px4_msgs::msg::VehicleCommand & msg)
{
  msg.timestamp = Px4TimestampNow();
  msg.target_system = target_system_;
  msg.target_component = 1;
  msg.source_system = 255;
  msg.from_external = true;
  float nan_float = std::numeric_limits<float>::quiet_NaN();
  msg.param1 = nan_float;
  msg.param2 = nan_float;
  msg.param3 = nan_float;
  msg.param4 = nan_float;
  msg.param5 = nan_float;
  msg.param6 = nan_float;
  msg.param7 = nan_float;
}

void Px4Comm::checkStatusReceived()
{
  auto now_ns{node_->now().nanoseconds()};
  const int64_t max_silence{2000000000};  // 2 seconds
  {
    std::unique_lock<std::mutex> lock(mtx_);
    if (connected_ && (now_ns - last_status_received_) > max_silence) {
      UAVCPP_FATAL("PX4 communication lost");
      connected_ = false;
    }
  }
  const int64_t max_ping{500000000};  // 500 ms
  if (now_ns - last_ping_offboard_ > max_ping) {
    pingOffboard();
  }
}

void Px4Comm::onVehicleStatus([[maybe_unused]] const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  last_status_received_ = node_->now().nanoseconds();
  if (!connected_) {
    target_system_ = msg->system_id;
    connected_ = true;
  }
}

void Px4Comm::onVehicleControlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
  if (!fcu_interface_) {
    return;
  }
  fcu_interface_->updateArmStatus(msg->flag_armed);
  fcu_interface_->updateOffboardStatus(msg->flag_control_offboard_enabled);
}

void Px4Comm::onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  if (!fcu_interface_) {
    return;
  }
  uav_cpp::types::OdometryStamped odom;
  odom = ros2_uav::utils::convert(*msg);
  fcu_interface_->updateOdometry(odom);
}

}  // namespace ros2_uav

// Copyright 2024 Damien SIX (damien@robotsix.net)
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

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <uav_cpp/parameters/param_container.hpp>
#include <px4_ros2/components/mode.hpp>
#include <uav_cpp/components/mode.hpp>
#include <ros2_uav_interfaces/msg/coordinate.hpp>
#include "ros2_uav_px4/utils/tf2_eigen.hpp"
#include <ros2_uav_interfaces/msg/disturbance.hpp>

namespace ros2_uav::modes
{
using uav_cpp::parameters::ParamContainer;
using uav_cpp::utils::Coordinate;
using px4_ros2::ModeBase;
using uav_ros2::utils::eigenNedToTf2Nwu;
using uav_ros2::utils::tf2FwuToEigenNed;

/**
 * @brief Concept that checks if ModeT is derived from uav_cpp::components::Mode.
 */
template<typename ModeT>
concept DerivedFromUavCppMode = requires(ModeT mode)
{
  [] < typename ... T > (uav_cpp::components::Mode<T...> &) {} (mode);
};

/**
 * @brief Interface class for UAV modes
 *
 * @tparam ModeT The mode type derived from uav_cpp::modes::Mode.
 */
template<typename ModeT>
class ModeInterface : public ModeBase, public ParamContainer
{
public:
  /**
   * @brief Constructs a new ModeInterface object.
   *
   * @param mode_settings Settings for the mode.
   * @param node Reference to the ROS2 node.
   */
  ModeInterface(const ModeBase::Settings & mode_settings, rclcpp::Node & node)
  : ModeBase(node, mode_settings),
    ParamContainer(),
    node_(node)
  {
    this->createMode();
    this->addChildContainer(this->mode_.get());
    vehicle_local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    vehicle_angular_velocity_ = std::make_shared<px4_ros2::OdometryAngularVelocity>(*this);
    vehicle_attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    // Add a subscription to the disturbance topic
    disturbance_sub_ = node_.create_subscription<ros2_uav_interfaces::msg::Disturbance>(
      "disturbance", 1, [this](const ros2_uav_interfaces::msg::Disturbance::SharedPtr msg) {
        tf2::Vector3 disturbance_constant(msg->constant.x, msg->constant.y, msg->constant.z);
        tf2::Vector3 disturbance_proportional(
          msg->proportional.x, msg->proportional.y, msg->proportional.z);
        mode_->setDisturbanceCoefficients(disturbance_constant, disturbance_proportional);
      });
  }

  /**
   * @brief Creates the uav_cpp::modes::Mode object.
   */
  void createMode()
  {
    std::string node_namespace = node_.get_namespace();
    if (node_namespace.empty()) {
      node_namespace = "/";
    }
    // Remove the leading slash
    node_namespace = node_namespace.substr(1);
    mode_ = std::make_shared<ModeT>(node_namespace);
  }

  /**
   * @brief Sets the setpoint for the mode.
   *
   * @param setpoint The setpoint to be set.
   */
  void setSetpoint(const ModeT::InputType & setpoint) {mode_->setInput(setpoint);}

  /**
   * @brief Set the TF Buffer for the mode.
   *
   * @param tf_buffer The TF Buffer to be set.
   */
  void setTfBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) {mode_->setTfBuffer(tf_buffer);}

protected:
  /**
   * @brief Function called when the mode is activated.
   */
  void onActivate() override
  {
    odometryUpdate();
    this->mode_->reset();
  }

  /**
   * @brief Function called when the mode is deactivated.
   */
  void onDeactivate() override {}

  /**
   * @brief Updates the odometry data.
   */
  void odometryUpdate()
  {
    auto position = vehicle_local_position_->positionNed();
    auto velocity = vehicle_local_position_->velocityNed();
    auto attitude = vehicle_attitude_->attitude();
    auto angular_velocity = vehicle_angular_velocity_->angularVelocityFrd();
    if (!mode_) {
      RCLCPP_ERROR(node_.get_logger(), "Mode not initialized");
      return;
    }
    this->mode_->setCurrentOdometry(
      eigenNedToTf2Nwu(position),
      eigenNedToTf2Nwu(attitude),
      eigenNedToTf2Nwu(velocity),
      eigenNedToTf2Nwu(angular_velocity));
  }

  rclcpp::Node & node_;  ///< Reference to the ROS2 node.
  std::shared_ptr<ModeT> mode_;  ///< The mode instance.

  rclcpp::Publisher<ros2_uav_interfaces::msg::Coordinate>::SharedPtr coordinate_publisher_;
  ///< The ROS2 publisher for the coordinates.
  rclcpp::Subscription<ros2_uav_interfaces::msg::Disturbance>::SharedPtr disturbance_sub_;
  ///< The ROS2 subscription for the disturbance.

  std::shared_ptr<px4_ros2::OdometryLocalPosition> vehicle_local_position_;
  ///< Shared pointer to vehicle local position.
  std::shared_ptr<px4_ros2::OdometryAngularVelocity> vehicle_angular_velocity_;
  ///< Shared pointer to vehicle angular velocity.
  std::shared_ptr<px4_ros2::OdometryAttitude> vehicle_attitude_;
  ///< Shared pointer to vehicle attitude.
};

}  // namespace ros2_uav::modes

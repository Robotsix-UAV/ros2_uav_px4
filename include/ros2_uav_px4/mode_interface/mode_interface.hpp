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
#include <vector>
#include <string>
#include <uav_cpp/parameters/param_container.hpp>
#include <px4_ros2/components/mode.hpp>
#include <uav_cpp/pipeline/control_pipeline.hpp>
#include <ros2_uav_interfaces/msg/disturbance.hpp>
#include "ros2_uav_px4/utils/frame_conversions.hpp"
#include "ros2_uav_px4/utils/type_conversions.hpp"
#include "ros2_uav_px4/utils/timestamp_validator.hpp"

namespace ros2_uav::modes
{
using uav_cpp::parameters::ParamContainer;
using uav_cpp::utils::Coordinate;
using px4_ros2::ModeBase;
using uav_ros2::utils::NedToNwu;
using uav_ros2::utils::NwuToNed;
using ros2_uav_interfaces::msg::ModeStatus;
using uav_ros2::utils::TimestampValidator;

/**
 * @brief Concept that checks if PipelineT is derived from uav_cpp::pipelines::ControlPipeline.
 */
template<typename PipelineT>
concept DerivedFromUavCppPipeline = requires(PipelineT pipeline)
{
  [] < typename ... T > (uav_cpp::pipelines::ControlPipeline<T...> &) {} (pipeline);
};

/**
 * @brief Interface class for UAV modes
 *
 * @tparam PipelineT The pipeline type derived from uav_cpp::pipelines::ControlPipeline.
 */
template<DerivedFromUavCppPipeline PipelineT>
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
    this->addChildContainer(this->pipeline_.get());
    vehicle_local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    vehicle_angular_velocity_ = std::make_shared<px4_ros2::OdometryAngularVelocity>(*this);
    vehicle_attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    addRequiredParameter<bool>("disturbance_observer.enable");
    // Add a subscription to the disturbance topic
    disturbance_sub_ = node_.create_subscription<ros2_uav_interfaces::msg::Disturbance>(
      "disturbance", 1, [this](const ros2_uav_interfaces::msg::Disturbance::SharedPtr msg) {
        bool disturbance_flag;
        getParameter("disturbance_observer.enable", disturbance_flag);
        if (!disturbance_flag) {
          return;
        }
        uav_cpp::types::DisturbanceCoefficientsStamped disturbance_coefficients;
        disturbance_coefficients = uav_ros2::utils::convert(*msg);
        pipeline_->setDisturbanceCoefficients(disturbance_coefficients);
      });
    auto mode_name = mode_settings.name;
    std::replace(mode_name.begin(), mode_name.end(), ' ', '_');
  }

  /**
   * @brief Creates the uav_cpp::modes::ControlPipeline object.
   */
  void createMode()
  {
    pipeline_ = std::make_shared<PipelineT>();
  }

  /**
   * @brief Sets the setpoint for the mode.
   *
   * @param setpoint The setpoint to be set.
   */
  void setSetpoint(const PipelineT::PipelineInputType & setpoint) {pipeline_->setInput(setpoint);}

  /**
   * @brief Check if the mode is idle.
   * @return True if the mode is idle, false otherwise.
   */
  bool isIdle() const {return pipeline_->isIdle();}

protected:
  /**
   * @brief Function called when the mode is activated.
   */
  void onActivate() override
  {
    odometryUpdate();
    this->pipeline_->reset();
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
    uav_cpp::types::OdometryStamped odometry;
    auto position = vehicle_local_position_->positionNed();
    auto velocity = vehicle_local_position_->velocityNed();
    auto attitude = vehicle_attitude_->attitude();
    auto angular_velocity = vehicle_angular_velocity_->angularVelocityFrd();
    odometry.frame_id = uav_cpp::enums::Frame::ODOM;
    odometry.timestamp = std::chrono::microseconds(vehicle_local_position_->last().timestamp);
    if (!timestamp_validator_.isValidTimestamp(odometry.timestamp.count())) {
      return;
    }
    odometry.position = NedToNwu(position.cast<double>());
    odometry.velocity = NedToNwu(velocity.cast<double>());
    odometry.attitude = NedToNwu(attitude.cast<double>());
    odometry.angular_velocity = NedToNwu(angular_velocity.cast<double>());
    if (!pipeline_) {
      UAVCPP_ERROR("ControlPipeline not initialized");
      return;
    }
    this->pipeline_->setCurrentOdometry(odometry);
  }

  rclcpp::Node & node_;  ///< Reference to the ROS2 node.
  std::shared_ptr<PipelineT> pipeline_;  ///< The uav_cpp pipeline.

  TimestampValidator timestamp_validator_;  ///< Timestamp validator for the odometry data.

  rclcpp::TimerBase::SharedPtr mode_status_timer_;  ///< Timer for the mode status.
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

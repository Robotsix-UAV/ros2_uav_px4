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
#include <px4_ros2/components/mode_executor.hpp>
#include <uav_cpp/utils/smart_pointer_base.hpp>
#include <uav_cpp/parameters/param_container.hpp>
#include "ros2_uav_px4/utils/origin_reset.hpp"

namespace ros2_uav::executors
{
/**
 * @brief Executor class to manage takeoff and mode execution for UAVs.
 */
class ExecutorTakeOff : public px4_ros2::ModeExecutorBase,
  public uav_cpp::parameters::ParamContainer,
  public uav_cpp::utils::SmartPointerBase<ExecutorTakeOff>,
  public uav_cpp::logger::LogTagHolder
{
public:
  /**
   * @brief Enum representing the states of the executor.
   */
  enum class State
  {
    ARM,        ///< State for arming the UAV.
    TAKEOFF,    ///< State for taking off the UAV.
    OWNED_MODE  ///< State for executing the owned mode.
  };

  /**
   * @brief Constructs a new ExecutorTakeOff object.
   *
   * @param node Reference to the ROS2 node.
   * @param owned_mode Reference to the owned mode.
   * @param origin_reset Reference to the origin reset object.
   */
  ExecutorTakeOff(
    rclcpp::Node & node, px4_ros2::ModeBase & owned_mode,
    std::shared_ptr<ros2_uav::utils::OriginReset> origin_reset)
  : px4_ros2::ModeExecutorBase(node,
      px4_ros2::ModeExecutorBase::Settings{Settings::Activation::ActivateAlways},
      owned_mode),
    uav_cpp::logger::LogTagHolder("Executor TakeOff"),
    origin_reset_(origin_reset)
  {
    addRequiredParameter<double>("takeoff.altitude");
  }

  /**
   * @brief Check if the executor is completed.
   */
  bool isCompleted() const {return current_state_ == State::OWNED_MODE;}

private:
  /**
   * @brief Function called when the executor is activated.
   */
  void onActivate() override
  {
    runState(State::ARM);
  }

  /**
   * @brief Function called when the executor is deactivated.
   *
   * @param reason The reason for deactivation.
   */
  void onDeactivate([[maybe_unused]] DeactivateReason reason) override
  {
  }

  /**
   * @brief Runs the specified state.
   *
   * @param state The state to run.
   */
  void runState(State state)
  {
    current_state_ = state;
    switch (state) {
      case State::ARM:
        if (!isArmed()) {
          origin_reset_->resetOrigin();
          UAVCPP_INFO_TAG(this, "Arming");
        }
        arm(
          [this](px4_ros2::Result result)
          {
            if (result == px4_ros2::Result::Success) {
              runState(State::TAKEOFF);
            }
          });
        break;
      case State::TAKEOFF:
        UAVCPP_INFO_TAG(this, "Taking off");
        takeoff(
          [this](px4_ros2::Result result)
          {
            if (result == px4_ros2::Result::Success) {
              runState(State::OWNED_MODE);
            }
          },
          2.0, 0);
        break;

      case State::OWNED_MODE:
        UAVCPP_INFO_TAG(this, "Owned mode");
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result) {return;});
        break;
    }
  }

  State current_state_{State::ARM};
  std::shared_ptr<ros2_uav::utils::OriginReset> origin_reset_;
};

}  // namespace ros2_uav::executors

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

#include <px4_ros2/components/mode_executor.hpp>
#include <uav_cpp/utils/smart_pointer_base.hpp>

namespace ros2_uav::executors
{
/**
 * @brief Executor class to manage arming and mode execution for UAVs.
 */
class ExecutorArm : public px4_ros2::ModeExecutorBase,
  public uav_cpp::utils::SmartPointerBase<ExecutorArm>
{
public:
  /**
   * @brief Enum representing the states of the executor.
   */
  enum class State
  {
    ARM,        ///< State for arming the UAV.
    OWNED_MODE  ///< State for executing the owned mode.
  };

  /**
   * @brief Constructs a new ExecutorArm object.
   *
   * @param node Reference to the ROS2 node.
   * @param owned_mode Reference to the owned mode.
   */
  ExecutorArm(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : px4_ros2::ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{false, true}, owned_mode)
  {}

  /**
   * @brief Function called when the executor is activated.
   */
  void onActivate() override
  {
    RCLCPP_DEBUG(node().get_logger(), "ExecutorArm activated");
    runState(State::ARM);
  }

  /**
   * @brief Function called when the executor is deactivated.
   *
   * @param reason The reason for deactivation.
   */
  void onDeactivate([[maybe_unused]] DeactivateReason reason) override
  {
    RCLCPP_DEBUG(node().get_logger(), "ExecutorArm deactivated");
  }

  /**
   * @brief Runs the specified state.
   *
   * @param state The state to run.
   */
  void runState(State state)
  {
    switch (state) {
      case State::ARM:
        RCLCPP_INFO(node().get_logger(), "[Arming Executor] Arming");
        arm(
          [this](px4_ros2::Result result)
          {
            if (result == px4_ros2::Result::Success) {
              runState(State::OWNED_MODE);
            }
          });
        break;
      case State::OWNED_MODE:
        RCLCPP_INFO(node().get_logger(), "[Arming Executor] Owned mode");
        scheduleMode(
          ownedMode().id(), [](px4_ros2::Result) {return;});
        break;
    }
  }
};

}  // namespace ros2_uav::executors

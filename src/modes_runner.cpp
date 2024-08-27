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

#include <tf2_ros/buffer.h>
#include <ros2_uav_parameters/parameter_client.hpp>
#include <ros2_uav_cpp/ros2_logger.hpp>
#include <uav_cpp/parameters/param_container.hpp>
#include <ros2_uav_interfaces/msg/pose_heading.hpp>

#include "ros2_uav_px4/executors/executor_arm.hpp"
#include "ros2_uav_px4/executors/executor_take_off.hpp"
#include "ros2_uav_px4/modes/spin.hpp"
#include "ros2_uav_px4/modes/position.hpp"
#include "ros2_uav_px4/modes/nlmpc_position.hpp"
#include "ros2_uav_px4/utils/uav_cpp_ros2_conversions.hpp"

using uav_cpp::parameters::ParameterMap;
using uav_cpp::parameters::ParamContainer;
using ros2_uav::utils::RosLoggerInterface;
using ros2_uav::modes::Spin;
using ros2_uav::modes::Position;
using ros2_uav::modes::NlmpcPosition;
using ros2_uav::executors::ExecutorArm;
using ros2_uav::executors::ExecutorTakeOff;
using ros2_uav_interfaces::msg::PoseHeading;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto mode_node = std::make_shared<rclcpp::Node>("mode_node");
  ParamContainer param_container;

  // Create tf buffer for the mode node
  auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(mode_node->get_clock());

  // Spin mode
  auto spin_mode = Spin::make_unique<>(*mode_node);
  auto spin_executor = ExecutorArm::make_unique<>(*mode_node, *spin_mode);
  param_container.addChildContainer(spin_mode.get());
  // arm executor has no parameters

  // Position mode
  auto position_mode = Position::make_shared<>(*mode_node);
  auto position_executor = ExecutorTakeOff::make_shared<>(*mode_node, *position_mode);
  position_mode->setTfBuffer(tf_buffer_);
  param_container.addChildContainer(position_mode.get());
  param_container.addChildContainer(position_executor.get());

  // NLMPC Position mode
  auto nlmpc_position_mode = NlmpcPosition::make_shared<>(*mode_node);
  auto nlmpc_position_executor = ExecutorTakeOff::make_shared<>(*mode_node, *nlmpc_position_mode);
  nlmpc_position_mode->setTfBuffer(tf_buffer_);
  param_container.addChildContainer(nlmpc_position_mode.get());
  param_container.addChildContainer(nlmpc_position_executor.get());
  
  // ROS callback for nlmpc positions setpoint
  auto nlmpc_position_setpoint_sub = mode_node->create_subscription<PoseHeading>(
    "command/pose_heading", 1,
    [nlmpc_position_mode, position_mode](const PoseHeading::SharedPtr msg) {
      position_mode->setSetpoint(uav_ros2::utils::convertFromRosMsg(*msg));
      nlmpc_position_mode->setSetpoint(uav_ros2::utils::convertFromRosMsg(*msg));
    });

  // Handle required parameters
  std::vector<std::string> required_parameters = param_container.getRequiredParameters();

  // Parameter client node
  auto parameter_client = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "mode_parameter_client", required_parameters);
  auto parameters = ParameterMap::make_shared<>(parameter_client->getParameters());

  // Set parameters
  param_container.setParameters(parameters);

  // Add nodes to executor
  executor.add_node(parameter_client);
  executor.add_node(mode_node);

  // Set the logger to ROS logger for the uav_cpp library
  auto uav_cpp_logger = rclcpp::get_logger("uav_cpp");
  auto ret = rcutils_logging_set_logger_level("uav_cpp", RCUTILS_LOG_SEVERITY_DEBUG);
  (void) ret;
  auto logger = std::make_shared<RosLoggerInterface>(uav_cpp_logger);
  uav_cpp::logger::Logger::setCustomLogger(logger);

  // Register modes in PX4
  spin_executor->doRegister();
  position_executor->doRegister();
  nlmpc_position_executor->doRegister();

  // Spin the nodes
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

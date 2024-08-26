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

#include <ros2_uav_cpp/ros2_logger.hpp>
#include <ros2_uav_parameters/parameter_client.hpp>
#include "ros2_uav_px4/model_identification/attitude_thrust_matcher.hpp"

using ros2_uav::utils::RosLoggerInterface;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto uav_cpp_logger = rclcpp::get_logger("uav_cpp");
  auto logger = std::make_shared<RosLoggerInterface>(uav_cpp_logger);
  uav_cpp::logger::Logger::setCustomLogger(logger);
  auto attitude_thrust_matcher = std::make_shared<ros2_uav::identification::AttitudeThrustMatcher>();
  auto required_parameters = attitude_thrust_matcher->getRequiredParameters();
  // Parameter client node
  auto parameter_client = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "mode_parameter_client", required_parameters);
  auto parameters = uav_cpp::parameters::ParameterMap::make_shared<>(parameter_client->getParameters());
  attitude_thrust_matcher->setParameters(parameters);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parameter_client);
  executor.add_node(attitude_thrust_matcher);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
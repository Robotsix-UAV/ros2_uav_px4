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

#include <uav_cpp/disturbance_observers/disturbance_observer.hpp>
#include <ros2_uav_parameters/parameter_client.hpp>
#include <ros2_uav_interfaces/msg/disturbance.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include "ros2_uav_px4/utils/type_conversions.hpp"

int main(int argc, char * argv[])
{
  using Vec3 = Eigen::Vector3d;
  using uav_cpp::types::VelocityStamped;
  using uav_cpp::types::AttitudeThrustStamped;
  using uav_cpp::types::DisturbanceCoefficientsStamped;
  rclcpp::init(argc, argv);
  uav_cpp::disturbance_observer::DisturbanceObserver disturbance_observer;
  std::vector<std::string> required_parameters = disturbance_observer.getRequiredParameters();
  required_parameters.push_back("model.thrust_constant_coefficient");
  required_parameters.push_back("model.thrust_linear_coefficient");
  required_parameters.push_back("model.thrust_quadratic_coefficient");

  // Parameter client node
  auto disturbance_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "disturbance_observer", required_parameters);
  auto parameters = uav_cpp::parameters::ParameterMap::make_shared<>(
    disturbance_node->getParameters());

  // Set parameters
  disturbance_observer.setParameters(parameters);

  // Set callback for the PX4 messages
  auto qos = rclcpp::QoS(1);
  qos.keep_last(1);
  qos.best_effort();
  qos.transient_local();

  auto callback =
    [&disturbance_observer,
      &disturbance_node](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      VelocityStamped velocity_stamped;
      velocity_stamped.vector = Vec3(msg->velocity[0], -msg->velocity[1], -msg->velocity[2]);
      velocity_stamped.timestamp = std::chrono::microseconds{msg->timestamp};
      disturbance_observer.setVelocity(velocity_stamped);
    };
  auto odometry_sub = disturbance_node->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "fmu/out/vehicle_odometry", qos, callback);

  auto callback_setpoint =
    [&disturbance_observer, &disturbance_node,
      &parameters](const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr msg) {
      uav_cpp::types::AttitudeThrustStamped inputs;
      // Conversion from FRD to FLU
      inputs.attitude = Eigen::Quaterniond(msg->q_d[0], msg->q_d[1], -msg->q_d[2], -msg->q_d[3]);
      inputs.thrust = -msg->thrust_body[2];
      inputs.timestamp = std::chrono::microseconds{msg->timestamp};
      double thrust_constant_coefficient, thrust_linear_coefficient, thrust_quadratic_coefficient;
      (*parameters)["model.thrust_constant_coefficient"]->getValue(thrust_constant_coefficient);
      (*parameters)["model.thrust_linear_coefficient"]->getValue(thrust_linear_coefficient);
      (*parameters)["model.thrust_quadratic_coefficient"]->getValue(thrust_quadratic_coefficient);
      inputs.thrust = thrust_constant_coefficient + thrust_linear_coefficient * inputs.thrust +
        thrust_quadratic_coefficient * inputs.thrust * inputs.thrust;
      disturbance_observer.setInputs(inputs);
    };
  auto setpoint_sub = disturbance_node->create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
    "fmu/in/vehicle_attitude_setpoint", 1, callback_setpoint);

  // Set the publisher for the disturbance observer
  auto disturbance_pub = disturbance_node->create_publisher<ros2_uav_interfaces::msg::Disturbance>(
    "disturbance", 1);
  auto disturbance_callback =
    [&disturbance_pub](const DisturbanceCoefficientsStamped & disturbance) {
      ros2_uav_interfaces::msg::Disturbance msg = uav_ros2::utils::convert(disturbance);
      disturbance_pub->publish(msg);
    };
  disturbance_observer.setComputationCallback(disturbance_callback);

  rclcpp::spin(disturbance_node);
  rclcpp::shutdown();
  return 0;
}

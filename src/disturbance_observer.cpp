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
#include <ros2_uav_cpp/ros2_logger.hpp>
#include <ros2_uav_interfaces/msg/disturbance.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

int main(int argc, char * argv[])
{
  using Vec3 = tf2::Vector3;
  rclcpp::init(argc, argv);
    uav_cpp::disturbance_observers::DisturbanceObserver disturbance_observer;
  std::vector<std::string> required_parameters = disturbance_observer.getRequiredParameters();
  required_parameters.push_back("px4.thrust_constant_coefficient");
  required_parameters.push_back("px4.thrust_linear_coefficient");
  required_parameters.push_back("px4.thrust_quadratic_coefficient");

  // Parameter client node
  auto disturbance_node = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "disturbance_observer", required_parameters);
  auto parameters = uav_cpp::parameters::ParameterMap::make_shared<>(disturbance_node->getParameters());

  // Set parameters
  disturbance_observer.setParameters(parameters);

  // Set callback for the PX4 messages
  auto qos = rclcpp::QoS(1);
  qos.keep_last(1);
  qos.best_effort();
  qos.transient_local();

  auto time_init = disturbance_node->now();
  auto callback = [&disturbance_observer, time_init, &disturbance_node](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    auto time_now = disturbance_node->now();
    auto dt = (time_now - time_init).seconds();
    tf2::Vector3 velocity(msg->velocity[0], -msg->velocity[1], -msg->velocity[2]);
    disturbance_observer.setVelocity(velocity, dt);
  };
  auto odometry_sub = disturbance_node->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "fmu/out/vehicle_odometry", qos, callback);

  auto callback_setpoint = [&disturbance_observer, time_init, &disturbance_node, &parameters](const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr msg) {
    uav_cpp::pipelines::AttitudeThrust inputs;
    // Conversion from FRD to FLU
    inputs.orientation = tf2::Quaternion(msg->q_d[1], -msg->q_d[2], -msg->q_d[3], msg->q_d[0]);
    inputs.thrust = -msg->thrust_body[2];
  double thrust_constant_coefficient, thrust_linear_coefficient, thrust_quadratic_coefficient;
  (*parameters)["px4.thrust_constant_coefficient"]->getValue(thrust_constant_coefficient);
  (*parameters)["px4.thrust_linear_coefficient"]->getValue(thrust_linear_coefficient);
  (*parameters)["px4.thrust_quadratic_coefficient"]->getValue(thrust_quadratic_coefficient);
  inputs.thrust = thrust_constant_coefficient + thrust_linear_coefficient * inputs.thrust + thrust_quadratic_coefficient * inputs.thrust * inputs.thrust;
  
    auto time_now = disturbance_node->now();
    auto dt = (time_now - time_init).seconds();
    disturbance_observer.setInputs(inputs, dt);
  };
  auto setpoint_sub = disturbance_node->create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
    "fmu/in/vehicle_attitude_setpoint", 1, callback_setpoint);

  // Set the publisher for the disturbance observer
  auto disturbance_pub = disturbance_node->create_publisher<ros2_uav_interfaces::msg::Disturbance>("disturbance", 1);
  auto disturbance_callback = [&disturbance_pub](const Vec3& constant_disturbance, const Vec3& proportional_disturbance) {
    ros2_uav_interfaces::msg::Disturbance msg;
    msg.constant.x = constant_disturbance.x();
    msg.constant.y = constant_disturbance.y();
    msg.constant.z = constant_disturbance.z();
    msg.proportional.x = proportional_disturbance.x();
    msg.proportional.y = proportional_disturbance.y();
    msg.proportional.z = proportional_disturbance.z();
    disturbance_pub->publish(msg);
  };
  disturbance_observer.setComputationCallback(disturbance_callback);

  // Set the logger to ROS logger for the uav_cpp library
  auto uav_cpp_logger = rclcpp::get_logger("uav_cpp");
  auto ret = rcutils_logging_set_logger_level("uav_cpp", RCUTILS_LOG_SEVERITY_DEBUG);
  (void) ret;
  auto logger = std::make_shared<ros2_uav::utils::RosLoggerInterface>(uav_cpp_logger);
  uav_cpp::logger::Logger::setCustomLogger(logger);

  rclcpp::spin(disturbance_node);
  rclcpp::shutdown();
  return 0;
}

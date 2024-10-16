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

#include <thread>
#include <uav_cpp/manager/core_manager.hpp>
#include <ros2_uav_parameters/parameter_client.hpp>
#include <uav_cpp/parameters/param_container.hpp>
#include <ros2_uav_interfaces/srv/user_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "uav_cpp/custom_pipelines/se3_position.hpp"
#include "uav_cpp/custom_pipelines/spin.hpp"
#include "uav_cpp/custom_pipelines/nlmpc_position.hpp"
#include "uav_cpp/custom_pipelines/nlmpc_waypoints.hpp"
#include "uav_cpp/custom_pipelines/stop.hpp"
#include <ros2_uav_interfaces/msg/waypoint_list.hpp>
#include <ros2_uav_interfaces/msg/pose_heading.hpp>
#include <ros2_uav_interfaces/msg/disturbance.hpp>
#include "ros2_uav_px4/px4_interface/px4_comm.hpp"
#include "ros2_uav_px4/utils/type_conversions.hpp"
#include "ros2_uav_px4/utils/origin_reset.hpp"

using ros2_uav_interfaces::srv::UserRequest;
using uav_cpp::manager::CoreManager;
using uav_cpp::fcu_interface::FcuInterface;
using uav_cpp::fcu_interface::InterfaceActions;
using uav_cpp::parameters::ParamContainer;
using ros2_uav::Px4Comm;
using uav_cpp::parameters::ParameterMap;
using ros2_uav_interfaces::msg::WaypointList;
using ros2_uav_interfaces::msg::PoseHeading;
using ros2_uav_interfaces::msg::Disturbance;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Logger config
  uav_cpp::logger::LogManager::getInstance("modes_runner.log");

  // Create the node
  auto controller_node = std::make_shared<rclcpp::Node>("controller_node");
  // Create the interface to the PX4 and configure
  auto px4_comm = std::make_shared<Px4Comm>(controller_node.get());
  UAVCPP_INFO("Waiting for connection to PX4");

  while (!px4_comm->isConnected()) {
    rclcpp::spin_some(controller_node);
  }
  auto target_system = px4_comm->getTargetSystem();
  auto origin_reset =
    std::make_shared<ros2_uav::utils::OriginReset>(*controller_node, target_system);
  UAVCPP_INFO("Waiting for global position to reset origin");
  while (!origin_reset->resetOrigin()) {
    rclcpp::spin_some(controller_node);
  }

  // Create the interface to the PX4 for the UAVCPP controller
  // Assign the actions to the interface
  InterfaceActions actions;
  actions.arm = [&px4_comm, &origin_reset](bool arm) {
      origin_reset->resetOrigin(); px4_comm->setArm(arm);
    };
  actions.offboard = [&px4_comm](bool offboard) {px4_comm->setOffboard(offboard);};
  actions.land = [&px4_comm]() {px4_comm->land();};
  actions.landHome = [&px4_comm]() {px4_comm->landHome();};
  actions.setAttitudeThrust = [&px4_comm](const uav_cpp::types::AttitudeThrustStamped & setpoint) {
      px4_comm->setAttitudeThrust(setpoint);
    };
  actions.setRatesThrust = [&px4_comm](const uav_cpp::types::RatesThrustStamped & setpoint) {
      px4_comm->setRatesThrust(setpoint);
    };
  actions.takeoff = [&px4_comm, &origin_reset]() {
      origin_reset->resetOrigin();
      px4_comm->takeoff();
    };
  auto fcu_interface = std::make_shared<FcuInterface>(actions);

  // Create the pipeline manager with the selected pipelines for our application
  using uav_cpp::pipelines::Spin;
  using uav_cpp::pipelines::Se3Position;
  using uav_cpp::pipelines::NlmpcPosition;
  using uav_cpp::pipelines::NlmpcWaypoints;
  using uav_cpp::pipelines::Stop;
  auto pipeline_manager = std::make_shared<uav_cpp::pipelines::PipelineManager<Spin, Se3Position,
      NlmpcPosition, NlmpcWaypoints, Stop>>();

  // Sets the trigger for the pipelines
  std::vector<std::string> triggers = {uav_cpp::fsm::events::Odometry{}.tag};
  pipeline_manager->getPipeline<"Spin">().setTriggerTags(triggers);
  pipeline_manager->getPipeline<"Se3Position">().setTriggerTags(triggers);
  pipeline_manager->getPipeline<"NlmpcPosition">().setTriggerTags(triggers);
  pipeline_manager->getPipeline<"NlmpcWaypoints">().setTriggerTags(triggers);
  pipeline_manager->getPipeline<"Stop">().setTriggerTags(triggers);

  // Create the core manager with the FCU interface and the pipeline manager
  CoreManager manager(fcu_interface, pipeline_manager);

  // Link the FCU interface taketo the PX4 interface
  px4_comm->setFcuInterface(fcu_interface);

  // Handle user requests in ROS2 to trigger the FSM events
  auto user_request_service = controller_node->create_service<UserRequest>(
    "user_request", [&manager](const std::shared_ptr<UserRequest::Request> request,
    [[maybe_unused]] std::shared_ptr<UserRequest::Response> response) {
      switch (request->action) {
        case UserRequest::Request::TAKE_OFF:
          manager.fsmEvent(uav_cpp::fsm::events::UserRequestTakeoff{});
          break;
        case UserRequest::Request::LAND:
          manager.fsmEvent(uav_cpp::fsm::events::UserRequestLand{});
          break;
        case UserRequest::Request::PIPELINE:
          {
            auto event = uav_cpp::fsm::events::RequestPipeline{};
            event.pipeline = request->pipeline_name;
            manager.fsmEvent(event);
            break;
          }
        default:
          break;
      }
    });

  // Handle the inputs from ROS2 topics
  auto waypoint_list_sub = controller_node->create_subscription<WaypointList>(
    "command/waypoints", 1, [&pipeline_manager](const WaypointList::SharedPtr msg) {
      pipeline_manager->setInput<"NlmpcWaypoints", uav_cpp::types::PoseSpeedVectorStamped>(
        ros2_uav::utils::convert(*msg));
    });

  auto pose_heading_sub = controller_node->create_subscription<PoseHeading>(
    "command/pose_heading", 1, [&pipeline_manager](const PoseHeading::SharedPtr msg) {
      pipeline_manager->setInput<"Se3Position", uav_cpp::types::PoseHeadingStamped>(
        ros2_uav::utils::convert(*msg));
      pipeline_manager->setInput<"NlmpcPosition", uav_cpp::types::PoseHeadingStamped>(
        ros2_uav::utils::convert(*msg));
    });

  // Disturbance observer
  auto disturbance_sub_ =
    controller_node->create_subscription<ros2_uav_interfaces::msg::Disturbance>(
    "disturbance", 1, [&manager](const ros2_uav_interfaces::msg::Disturbance::SharedPtr msg) {
      uav_cpp::types::DisturbanceCoefficientsStamped disturbance_coefficients;
      disturbance_coefficients = ros2_uav::utils::convert(*msg);
      manager.setDisturbanceCoefficients(disturbance_coefficients);
    });

  // Get the required parameters for the manager
  std::vector<std::string> required_parameters = manager.getRequiredParameters();

  // Create the parameter client with the required parameters (ROS2)
  auto parameter_client = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "mode_parameter_client", required_parameters);
  auto parameters = ParameterMap::make_shared<>(parameter_client->getParameters());
  // Set the parameters to the manager
  manager.setParameters(parameters);

  // Create a publisher for the fsm state
  auto fsm_state_publisher =
    controller_node->create_publisher<std_msgs::msg::String>("fsm_state", 1);
  auto state_pub_timer = controller_node->create_wall_timer(
    std::chrono::milliseconds(10), [&manager, &fsm_state_publisher]() {
      auto msg = std_msgs::msg::String();
      msg.data = manager.getFsmState();
      fsm_state_publisher->publish(msg);
    });

  // Execute the nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_node);
  executor.add_node(parameter_client);

  executor.spin();

  // Shutdown ROS2
  uav_cpp::logger::LogManager::getInstance().flushAllSinks();
  rclcpp::shutdown();

  return 0;
}

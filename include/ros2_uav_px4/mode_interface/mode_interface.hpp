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

namespace ros2_uav::modes
{
using uav_cpp::parameters::ParamContainer;
using uav_cpp::utils::Coordinate;
using px4_ros2::ModeBase;

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
template<DerivedFromUavCppMode ModeT>
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
    std::string node_namespace = node_.get_namespace();
    if (node_namespace.empty()) {
      node_namespace = "/";
    }
    // Remove the leading slash
    node_namespace = node_namespace.substr(1);
    mode_.setUavName(node_namespace);

    coordinate_publisher_ = node_.create_publisher<ros2_uav_interfaces::msg::Coordinate>(
      "debug/coordinates", 20);
  }

  /**
   * @brief Sets the setpoint for the mode.
   *
   * @param setpoint The setpoint to be set.
   */
  void setSetpoint(const ModeT::InputType & setpoint) {mode_.setInput(setpoint);}

  /**
   * @brief Set the TF Buffer for the mode.
   *
   * @param tf_buffer The TF Buffer to be set.
   */
  void setTfBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) {mode_.setTfBuffer(tf_buffer);}

protected:
  rclcpp::Node & node_;  ///< Reference to the ROS2 node.
  ModeT mode_;  ///< The mode instance.

  rclcpp::Publisher<ros2_uav_interfaces::msg::Coordinate>::SharedPtr coordinate_publisher_;
  ///< The ROS2 publisher for the coordinates.
};

}  // namespace ros2_uav::modes

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
#include <uav_cpp/parameters/param_container.hpp>
#include <px4_ros2/components/mode.hpp>
#include <uav_cpp/modes/mode.hpp>
#include <ros2_uav_interfaces/msg/coordinate.hpp>

namespace ros2_uav::modes
{
using uav_cpp::parameters::ParamContainer;
using uav_cpp::utils::Coordinate;
using px4_ros2::ModeBase;

template<typename ModeT>
concept DerivedFromUavCppMode = requires{
  std::is_base_of_v<uav_cpp::modes::Mode<typename ModeT::TrackerType,
    typename ModeT::ControllerType>, ModeT>;
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
    coordinate_publisher_ = node_.create_publisher<ros2_uav_interfaces::msg::Coordinate>(
      "debug/coordinates", 1);
  }

  /**
   * @brief Sets the setpoint for the mode.
   *
   * @param setpoint The setpoint to be set.
   */
  void setSetpoint(const ModeT::TrackerType::SetPointType & setpoint) {mode_.setSetpoint(setpoint);}

  /**
   * @brief Set the TF Buffer for the mode.
   *
   * @param tf_buffer The TF Buffer to be set.
   */
  void setTfBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) {mode_.setTfBuffer(tf_buffer);}

  /**
   * @brief Publishes the coordinates for debug.
   *
   * @param coordinates The coordinates to be published.
   */
  void publishCoordinates(const std::vector<Coordinate::SharedPtr> & coordinates)
  {
    ros2_uav_interfaces::msg::Coordinate coordinate_msg;
    ros2_uav_interfaces::msg::FloatArray float_array_msg;
    float final_time;
    std::vector<double> sample_times;
    for (const auto & coordinate : coordinates) {
      coordinate_msg.name = coordinate->getName();
      final_time = coordinate->getFinalTime();
      sample_times.clear();
      for (double sample_time = 0.0; sample_time <= final_time; sample_time += 0.01) {
        sample_times.push_back(sample_time);
      }
      for (uint8_t derivative = 0; derivative <= coordinate->getMaxDerivativeOrder();
        ++derivative)
      {
        if (derivative < coordinate->getMinDerivativeOrder()) {
          coordinate_msg.derivatives.push_back(ros2_uav_interfaces::msg::FloatArray());
          continue;
        }
        coordinate->getTrajectory(sample_times, derivative, float_array_msg.data);
        coordinate_msg.derivatives.push_back(float_array_msg);
      }
      coordinate_msg.frame_id = mode_.getControllerReferenceFrame();
      coordinate_msg.timestamps = sample_times;
      coordinate_publisher_->publish(coordinate_msg);
    }
  }

protected:
  rclcpp::Node & node_;  ///< Reference to the ROS2 node.
  ModeT mode_;  ///< The mode instance.

  rclcpp::Publisher<ros2_uav_interfaces::msg::Coordinate>::SharedPtr coordinate_publisher_;
  ///< The ROS2 publisher for the coordinates.
};

}  // namespace ros2_uav::modes

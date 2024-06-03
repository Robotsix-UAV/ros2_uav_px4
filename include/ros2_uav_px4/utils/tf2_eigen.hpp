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

#include <Eigen/Geometry>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace uav_ros2::utils
{

/**
 * @brief Converts an Eigen vector from NED (North-East-Down) coordinate system to a tf2 vector in NWU (North-West-Up) coordinate system.
 *
 * @param eigen_vector The Eigen vector in NED coordinate system.
 * @return tf2::Vector3 The tf2 vector in NWU coordinate system.
 */
tf2::Vector3 eigenNedToTf2Nwu(const Eigen::Vector3f & eigen_vector)
{
  return tf2::Vector3(eigen_vector.x(), -eigen_vector.y(), -eigen_vector.z());
}

/**
 * @brief Converts a tf2 vector from NWU (North-West-Up) coordinate system to an Eigen vector in NED (North-East-Down) coordinate system.
 *
 * @param tf2_vector The tf2 vector in NWU coordinate system.
 * @return Eigen::Vector3f The Eigen vector in NED coordinate system.
 */
Eigen::Vector3f tf2FwuToEigenNed(const tf2::Vector3 & tf2_vector)
{
  return Eigen::Vector3f(tf2_vector.x(), -tf2_vector.y(), -tf2_vector.z());
}

/**
 * @brief Converts an Eigen quaternion from NED (North-East-Down) coordinate system to a tf2 quaternion in NWU (North-West-Up) coordinate system.
 *
 * @param eigen_quaternion The Eigen quaternion in NED coordinate system.
 * @return tf2::Quaternion The tf2 quaternion in NWU coordinate system.
 */
tf2::Quaternion eigenNedToTf2Nwu(const Eigen::Quaternionf & eigen_quaternion)
{
  return tf2::Quaternion(
    eigen_quaternion.x(), -eigen_quaternion.y(),
    -eigen_quaternion.z(), eigen_quaternion.w());
}

/**
 * @brief Converts a tf2 quaternion from NWU (North-West-Up) coordinate system to an Eigen quaternion in NED (North-East-Down) coordinate system.
 *
 * @param tf2_quaternion The tf2 quaternion in NWU coordinate system.
 * @return Eigen::Quaternionf The Eigen quaternion in NED coordinate system.
 */
Eigen::Quaternionf tf2FwuToEigenNed(const tf2::Quaternion & tf2_quaternion)
{
  return Eigen::Quaternionf(
    tf2_quaternion.w(), tf2_quaternion.x(),
    -tf2_quaternion.y(), -tf2_quaternion.z());
}

}  // namespace uav_ros2::utils

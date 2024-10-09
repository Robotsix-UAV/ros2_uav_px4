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

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace ros2_uav::utils
{
Eigen::Vector3d NedToNwu(const Eigen::Vector3d & ned)
{
  return Eigen::Vector3d(ned.x(), -ned.y(), -ned.z());
}

Eigen::Vector3d NwuToNed(const Eigen::Vector3d & nwu)
{
  return Eigen::Vector3d(nwu.x(), -nwu.y(), -nwu.z());
}

Eigen::Quaterniond NedToNwu(const Eigen::Quaterniond & ned)
{
  return Eigen::Quaterniond(ned.w(), ned.x(), -ned.y(), -ned.z());
}

Eigen::Quaterniond NwuToNed(const Eigen::Quaterniond & nwu)
{
  return Eigen::Quaterniond(nwu.w(), nwu.x(), -nwu.y(), -nwu.z());
}

}  // namespace ros2_uav::utils

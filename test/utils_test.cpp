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

#include <gtest/gtest.h>
#include "ros2_uav_px4/utils/uav_cpp_ros2_conversions.hpp"
#include "ros2_uav_px4/utils/tf2_eigen.hpp"

using uav_ros2::utils::convertToSetpoint;
using uav_ros2::utils::eigenNedToTf2Nwu;
using uav_ros2::utils::tf2FwuToEigenNed;

// Testing vector conversions from NED to NWU and back
TEST(VectorConversionTests, PositiveValues) {
  Eigen::Vector3f eigen_vector(1.0, 2.0, 3.0);
  tf2::Vector3 converted_vector = eigenNedToTf2Nwu(eigen_vector);
  EXPECT_FLOAT_EQ(converted_vector.x(), 1.0);
  EXPECT_FLOAT_EQ(converted_vector.y(), -2.0);
  EXPECT_FLOAT_EQ(converted_vector.z(), -3.0);

  Eigen::Vector3f back_converted_vector = tf2FwuToEigenNed(converted_vector);
  EXPECT_FLOAT_EQ(back_converted_vector.x(), 1.0);
  EXPECT_FLOAT_EQ(back_converted_vector.y(), 2.0);
  EXPECT_FLOAT_EQ(back_converted_vector.z(), 3.0);
}

TEST(VectorConversionTests, ZeroValues) {
  Eigen::Vector3f eigen_vector(0.0, 0.0, 0.0);
  tf2::Vector3 converted_vector = eigenNedToTf2Nwu(eigen_vector);
  EXPECT_FLOAT_EQ(converted_vector.x(), 0.0);
  EXPECT_FLOAT_EQ(converted_vector.y(), 0.0);
  EXPECT_FLOAT_EQ(converted_vector.z(), 0.0);

  Eigen::Vector3f back_converted_vector = tf2FwuToEigenNed(converted_vector);
  EXPECT_FLOAT_EQ(back_converted_vector.x(), 0.0);
  EXPECT_FLOAT_EQ(back_converted_vector.y(), 0.0);
  EXPECT_FLOAT_EQ(back_converted_vector.z(), 0.0);
}

// Testing quaternion conversions from NED to NWU and back
TEST(QuaternionConversionTests, IdentityQuaternion) {
  Eigen::Quaternionf eigen_quaternion(1.0, 0.0, 0.0, 0.0);   // Identity Quaternion
  tf2::Quaternion converted_quaternion = eigenNedToTf2Nwu(eigen_quaternion);
  EXPECT_FLOAT_EQ(converted_quaternion.w(), 1.0);
  EXPECT_FLOAT_EQ(converted_quaternion.x(), 0.0);
  EXPECT_FLOAT_EQ(converted_quaternion.y(), 0.0);
  EXPECT_FLOAT_EQ(converted_quaternion.z(), 0.0);

  Eigen::Quaternion<float> back_converted_quaternion = tf2FwuToEigenNed(converted_quaternion);
  EXPECT_FLOAT_EQ(back_converted_quaternion.w(), 1.0);
  EXPECT_FLOAT_EQ(back_converted_quaternion.x(), 0.0);
  EXPECT_FLOAT_EQ(back_converted_quaternion.y(), 0.0);
  EXPECT_FLOAT_EQ(back_converted_quaternion.z(), 0.0);
}

// Test ROS Message to Setpoint Conversion
TEST(ConversionToSetpointTests, DefaultMessage) {
  ros2_uav_interfaces::msg::PoseHeading msg;   // Default initialized
  uav_cpp::types::PoseHeading setpoint = uav_ros2::utils::convertToSetpoint(msg);

  EXPECT_EQ(setpoint.frame_id, msg.header.frame_id);
  EXPECT_FLOAT_EQ(setpoint.position.x(), msg.position.x);
  EXPECT_FLOAT_EQ(setpoint.position.y(), msg.position.y);
  EXPECT_FLOAT_EQ(setpoint.position.z(), msg.position.z);
  EXPECT_FLOAT_EQ(setpoint.heading, msg.heading);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

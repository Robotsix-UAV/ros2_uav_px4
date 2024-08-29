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

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;

/**
 * @brief Class to publish the odometry data from the PX4 flight controller as a TF.
 */
class Px4TfPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Px4 Tf Publisher object.
   */
  Px4TfPublisher()
  : Node("px4_tf_publisher")
  {
    std::string node_namespace = this->get_namespace();
    if (node_namespace == "/") {
      uav_name_ = "uav";
    } else {
      uav_name_ = node_namespace.substr(1);
    }

    rclcpp::QoS qos(1);
    qos.keep_last(1);
    qos.best_effort();
    qos.transient_local();
    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry", qos,
      std::bind(&Px4TfPublisher::odom_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  /**
   * @brief Callback function to publish the odometry data as a TF.
   */
  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = uav_name_ + "/odom";
    t.child_frame_id = uav_name_ + "/base_link";

    t.transform.translation.x = msg->position[0];
    t.transform.translation.y = -msg->position[1];
    t.transform.translation.z = -msg->position[2];

    t.transform.rotation.x = msg->q[1];
    t.transform.rotation.y = -msg->q[2];
    t.transform.rotation.z = -msg->q[3];
    t.transform.rotation.w = msg->q[0];

    tf_broadcaster_->sendTransform(t);
  }

  std::string uav_name_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TfPublisher>());
  rclcpp::shutdown();
  return 0;
}

<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.11.0" doxygen_gitid="9b424b03c9833626cd435af22a444888fbbb192d">
  <compound kind="class">
    <name>ros2_uav::utils::OriginReset</name>
    <filename>classros2__uav_1_1utils_1_1OriginReset.html</filename>
    <member kind="function">
      <type></type>
      <name>OriginReset</name>
      <anchorfile>classros2__uav_1_1utils_1_1OriginReset.html</anchorfile>
      <anchor>a5e6168a014733c9c7fa9298c34c7f2ff</anchor>
      <arglist>(rclcpp::Node &amp;node, int target_system_id)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>resetOrigin</name>
      <anchorfile>classros2__uav_1_1utils_1_1OriginReset.html</anchorfile>
      <anchor>a3f4b94d6d911461ac30b7f7c6e29a9f4</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::Px4Comm</name>
    <filename>classros2__uav_1_1Px4Comm.html</filename>
    <base>Px4CommBase</base>
    <member kind="function">
      <type></type>
      <name>Px4Comm</name>
      <anchorfile>classros2__uav_1_1Px4Comm.html</anchorfile>
      <anchor>ae048c951717f0dae81ad85fa768a0cd0</anchor>
      <arglist>(rclcpp::Node *node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Px4Comm</name>
      <anchorfile>classros2__uav_1_1Px4Comm.html</anchorfile>
      <anchor>a8e59d0106355272d7b9ed58abe16ee59</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Px4CommBase</name>
    <filename>classPx4CommBase.html</filename>
    <member kind="function">
      <type></type>
      <name>Px4CommBase</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a4b6a6f14d282043e2c463eb10f879948</anchor>
      <arglist>(rclcpp::Node *node)</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="virtual">
      <type>virtual void</type>
      <name>onVehicleOdometry</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a866209df54f57adda6d5525a2bd2b588</anchor>
      <arglist>(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="virtual">
      <type>virtual void</type>
      <name>onVehicleStatus</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a47f326da05fcddf4bb95f85829a481a8</anchor>
      <arglist>(const px4_msgs::msg::VehicleStatus::SharedPtr msg)</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="virtual">
      <type>virtual void</type>
      <name>onVehicleControlMode</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>ab00a775e5e564fca0abc6d68187f262a</anchor>
      <arglist>(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Node *</type>
      <name>node_</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a88fc385bc1b173df113c18b8f76a3998</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Subscription&lt; px4_msgs::msg::VehicleOdometry &gt;::SharedPtr</type>
      <name>vehicle_odometry_sub</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>ad687d1fb51b6b537f80d230c93961786</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Subscription&lt; px4_msgs::msg::VehicleStatus &gt;::SharedPtr</type>
      <name>vehicle_status_sub</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a1ccfd0073fb868a3de45540f6a900858</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Subscription&lt; px4_msgs::msg::VehicleControlMode &gt;::SharedPtr</type>
      <name>vehicle_control_mode_sub</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a162b1bf4f8c067ac668e4e62ce0c185a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Publisher&lt; px4_msgs::msg::OffboardControlMode &gt;::SharedPtr</type>
      <name>offboard_control_mode_pub</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a96a8d73f9bf99ce394e5dcdc3b45ad07</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Publisher&lt; px4_msgs::msg::VehicleAttitudeSetpoint &gt;::SharedPtr</type>
      <name>vehicle_attitude_setpoint_pub</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a427514943074f90123885d747430ba72</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Publisher&lt; px4_msgs::msg::VehicleRatesSetpoint &gt;::SharedPtr</type>
      <name>vehicle_rates_setpoint_pub</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>aa378f85974009f08f28747d2abc39823</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Publisher&lt; px4_msgs::msg::VehicleCommand &gt;::SharedPtr</type>
      <name>vehicle_command_pub</name>
      <anchorfile>classPx4CommBase.html</anchorfile>
      <anchor>a33a9395e50beb3cdf38f4432da33ab99</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::identification::ThrustMatcher</name>
    <filename>classros2__uav_1_1identification_1_1ThrustMatcher.html</filename>
    <member kind="function">
      <type>void</type>
      <name>actuatorMotorsCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1ThrustMatcher.html</anchorfile>
      <anchor>a5701fab1f4772ee23e75ebf3f09690b4</anchor>
      <arglist>(const ActuatorMotors::SharedPtr actuator_motors)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>odometryCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1ThrustMatcher.html</anchorfile>
      <anchor>a8c1d17cadbb1f89f99b20438651ce124</anchor>
      <arglist>(const VehicleOdometry::SharedPtr odometry)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>accelerationCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1ThrustMatcher.html</anchorfile>
      <anchor>a31109ee90122d807466513dfb45f94fa</anchor>
      <arglist>(const VehicleAcceleration::SharedPtr acceleration)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>controlModeCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1ThrustMatcher.html</anchorfile>
      <anchor>a89c3fa8526215afb53f62c3d2d87e67b</anchor>
      <arglist>(const VehicleControlMode::SharedPtr control_mode)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::utils::TimestampValidator</name>
    <filename>classros2__uav_1_1utils_1_1TimestampValidator.html</filename>
    <member kind="function">
      <type></type>
      <name>TimestampValidator</name>
      <anchorfile>classros2__uav_1_1utils_1_1TimestampValidator.html</anchorfile>
      <anchor>a045f775c1390e7aeac565f37b4dd9e8f</anchor>
      <arglist>(int64_t tolerance_ns=1000000000, int max_outliers=10)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isValidTimestamp</name>
      <anchorfile>classros2__uav_1_1utils_1_1TimestampValidator.html</anchorfile>
      <anchor>a7ca653a3e0c624bf0256fcea3bc03fac</anchor>
      <arglist>(int64_t timestamp_ns)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>resetReference</name>
      <anchorfile>classros2__uav_1_1utils_1_1TimestampValidator.html</anchorfile>
      <anchor>a43296d11d47d20575eb92bee09205674</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav</name>
    <filename>namespaceros2__uav.html</filename>
    <namespace>ros2_uav::debug</namespace>
    <namespace>ros2_uav::identification</namespace>
    <namespace>ros2_uav::utils</namespace>
    <class kind="class">ros2_uav::Px4Comm</class>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::debug</name>
    <filename>namespaceros2__uav_1_1debug.html</filename>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::identification</name>
    <filename>namespaceros2__uav_1_1identification.html</filename>
    <class kind="class">ros2_uav::identification::ThrustMatcher</class>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::utils</name>
    <filename>namespaceros2__uav_1_1utils.html</filename>
    <class kind="class">ros2_uav::utils::OriginReset</class>
    <class kind="class">ros2_uav::utils::TimestampValidator</class>
    <member kind="function">
      <type>uav_cpp::types::Frame</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>a4dc8f3c52969b92885e4cadc34d14924</anchor>
      <arglist>(const std::string &amp;frame_id)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>a32831ac19257fcc5c5cd129524a45141</anchor>
      <arglist>(const uav_cpp::types::Frame &amp;frame_id)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::types::PoseHeadingStamped</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>a52cc96bc346c4a2cf503457c57f5f423</anchor>
      <arglist>(const ros2_uav_interfaces::msg::PoseHeading &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::types::DisturbanceCoefficientsStamped</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>ae8c420cd2484b81f1b4a8606d189a623</anchor>
      <arglist>(const ros2_uav_interfaces::msg::Disturbance &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>ros2_uav_interfaces::msg::Disturbance</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>a6d25827a289591cbcdbf8b1133bb3caa</anchor>
      <arglist>(const uav_cpp::types::DisturbanceCoefficientsStamped &amp;disturbance)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::types::AttitudeThrustStamped</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>a9314aef9dbf808b65c89dfa8e34d442d</anchor>
      <arglist>(const px4_msgs::msg::VehicleAttitudeSetpoint &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::types::OdometryStamped</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>a060d2a65a2c88fb87508e469113ee0cd</anchor>
      <arglist>(const px4_msgs::msg::VehicleOdometry &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::types::ThrustStamped</type>
      <name>convert</name>
      <anchorfile>namespaceros2__uav_1_1utils.html</anchorfile>
      <anchor>ad49a4e907ba9d315cdf6a72523b7348d</anchor>
      <arglist>(const px4_msgs::msg::VehicleThrustSetpoint &amp;msg)</arglist>
    </member>
  </compound>
</tagfile>

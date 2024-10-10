<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.11.0" doxygen_gitid="9b424b03c9833626cd435af22a444888fbbb192d">
  <compound kind="class">
    <name>ros2_uav::modes::AttitudeThrustMode</name>
    <filename>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</filename>
    <templarg>DerivedFromAttitudeThrustMode PipelineT</templarg>
    <base>ros2_uav::modes::ModeInterface&lt; PipelineT &gt;</base>
    <member kind="function">
      <type></type>
      <name>AttitudeThrustMode</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>a93baecac6fdc6436b7c4e2ac1a799d69</anchor>
      <arglist>(const ModeBase::Settings &amp;mode_settings, rclcpp::Node &amp;node)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::utils::ButterworthFilter</name>
    <filename>classros2__uav_1_1utils_1_1ButterworthFilter.html</filename>
    <member kind="function">
      <type></type>
      <name>ButterworthFilter</name>
      <anchorfile>classros2__uav_1_1utils_1_1ButterworthFilter.html</anchorfile>
      <anchor>a55f09ca3ce85f7e5f2b17c4689e8f6a6</anchor>
      <arglist>(double cutoff_frequency, double sampling_frequency)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>filtfilt</name>
      <anchorfile>classros2__uav_1_1utils_1_1ButterworthFilter.html</anchorfile>
      <anchor>a9ef948e62081f43383cba96edfd71e81</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;input)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>filter</name>
      <anchorfile>classros2__uav_1_1utils_1_1ButterworthFilter.html</anchorfile>
      <anchor>abc1b53ed41c896f3c71f2ccfe854a98e</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;input)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::utils::DerivativeFilter</name>
    <filename>classros2__uav_1_1utils_1_1DerivativeFilter.html</filename>
    <member kind="function">
      <type></type>
      <name>DerivativeFilter</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>a3eaba73d7bbc4c0fa5136adc7a37071f</anchor>
      <arglist>(double cutoff_frequency, double sampling_frequency)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addData</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>a12c5ef16de855ec78aa961c90cd4fb2e</anchor>
      <arglist>(double value)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>computeDerivative</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>a1de6e075d6368a610c7e73d59d92b27d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>process</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>a1431022e3a75df25c69fc99d213af78c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>process</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>a125806fe3d9f1b6d59f467f1e163df06</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;derivative, std::vector&lt; double &gt; &amp;derivative_filtered)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::utils::DerivativeFilter3D</name>
    <filename>classros2__uav_1_1utils_1_1DerivativeFilter3D.html</filename>
    <member kind="function">
      <type></type>
      <name>DerivativeFilter3D</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>aac919e5d6bc2800f0810625805fddefb</anchor>
      <arglist>(double cutoff_frequency, double sampling_frequency)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addData</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>a84e1481d339ffa5728671534464f70bf</anchor>
      <arglist>(const tf2::Vector3 &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; tf2::Vector3 &gt;</type>
      <name>process</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>a556eae6ee057a5f7d5bd8b5711489d6c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>process</name>
      <anchorfile>classros2__uav_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>a5957b9e8efd43add779f537b0bac76ca</anchor>
      <arglist>(std::vector&lt; tf2::Vector3 &gt; &amp;derivative, std::vector&lt; tf2::Vector3 &gt; &amp;derivative_filtered)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::executors::ExecutorArm</name>
    <filename>classros2__uav_1_1executors_1_1ExecutorArm.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>State</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorArm.html</anchorfile>
      <anchor>a7abe1b3ae997d55b267ddfd06ff5f6d5</anchor>
      <arglist></arglist>
      <enumvalue file="classros2__uav_1_1executors_1_1ExecutorArm.html" anchor="a7abe1b3ae997d55b267ddfd06ff5f6d5a47f45e65244c17ec9fa8771a5c6d60e1">ARM</enumvalue>
      <enumvalue file="classros2__uav_1_1executors_1_1ExecutorArm.html" anchor="a7abe1b3ae997d55b267ddfd06ff5f6d5a4dab4c093bafb64e050a9576c510adac">OWNED_MODE</enumvalue>
    </member>
    <member kind="function">
      <type></type>
      <name>ExecutorArm</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorArm.html</anchorfile>
      <anchor>a2d60ee77878f8f1070799ba58cf58f04</anchor>
      <arglist>(rclcpp::Node &amp;node, px4_ros2::ModeBase &amp;owned_mode)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isCompleted</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorArm.html</anchorfile>
      <anchor>a6eb4694d86da71c7bdc8c59f4b62cc60</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::executors::ExecutorLand</name>
    <filename>classros2__uav_1_1executors_1_1ExecutorLand.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>State</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorLand.html</anchorfile>
      <anchor>a06776e35262e1779cc7cb3085cab0c37</anchor>
      <arglist></arglist>
      <enumvalue file="classros2__uav_1_1executors_1_1ExecutorLand.html" anchor="a06776e35262e1779cc7cb3085cab0c37a479a809c0b6eaaefd3b1df16f976df06">LAND</enumvalue>
      <enumvalue file="classros2__uav_1_1executors_1_1ExecutorLand.html" anchor="a06776e35262e1779cc7cb3085cab0c37a4dab4c093bafb64e050a9576c510adac">OWNED_MODE</enumvalue>
    </member>
    <member kind="function">
      <type></type>
      <name>ExecutorLand</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorLand.html</anchorfile>
      <anchor>a0c1439c91ef0eae3e4dffb59ba4830b4</anchor>
      <arglist>(rclcpp::Node &amp;node, px4_ros2::ModeBase &amp;owned_mode)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isCompleted</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorLand.html</anchorfile>
      <anchor>a4768e3423ba524cd3cb12ffddf259ee5</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::executors::ExecutorTakeOff</name>
    <filename>classros2__uav_1_1executors_1_1ExecutorTakeOff.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>State</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorTakeOff.html</anchorfile>
      <anchor>ac05ed663a3acc3417532169516af7f63</anchor>
      <arglist></arglist>
      <enumvalue file="classros2__uav_1_1executors_1_1ExecutorTakeOff.html" anchor="ac05ed663a3acc3417532169516af7f63a47f45e65244c17ec9fa8771a5c6d60e1">ARM</enumvalue>
      <enumvalue file="classros2__uav_1_1executors_1_1ExecutorTakeOff.html" anchor="ac05ed663a3acc3417532169516af7f63a8fabc74a4ed0781d663336cbf8c9c53d">TAKEOFF</enumvalue>
      <enumvalue file="classros2__uav_1_1executors_1_1ExecutorTakeOff.html" anchor="ac05ed663a3acc3417532169516af7f63a4dab4c093bafb64e050a9576c510adac">OWNED_MODE</enumvalue>
    </member>
    <member kind="function">
      <type></type>
      <name>ExecutorTakeOff</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorTakeOff.html</anchorfile>
      <anchor>a3c4f8b8deb1a4cb4db789259282bea4c</anchor>
      <arglist>(rclcpp::Node &amp;node, px4_ros2::ModeBase &amp;owned_mode, std::shared_ptr&lt; ros2_uav::utils::OriginReset &gt; origin_reset)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isCompleted</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorTakeOff.html</anchorfile>
      <anchor>a531e4b7841a779697d392aeee574db0a</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::modes::ModeInterface</name>
    <filename>classros2__uav_1_1modes_1_1ModeInterface.html</filename>
    <templarg>DerivedFromUavCppPipeline PipelineT</templarg>
    <member kind="function">
      <type></type>
      <name>ModeInterface</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a2b7891d1f6741b5332eb938b0cbf9686</anchor>
      <arglist>(const ModeBase::Settings &amp;mode_settings, rclcpp::Node &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>createMode</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a29f35537614ae21f3876cfa63fb823d5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setSetpoint</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a5544604a61a1c7fa0176550d304e09b7</anchor>
      <arglist>(const PipelineT::PipelineInputType &amp;setpoint)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isIdle</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a6f31522ed70a070a79ed5711aaaea63b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>onActivate</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>aacf3d7664d0e2a41d1bfeabc39d3081c</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>onDeactivate</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a849397823fa6776114e28be4c8244b86</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>odometryUpdate</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>aee3170d2ed0452d08f9af0afbc0e4f48</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Node &amp;</type>
      <name>node_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>afac5611984bf54d4ff95cb6c46d32962</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; PipelineT &gt;</type>
      <name>pipeline_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a14f4b44257f4641c81679c07ee1e951b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>TimestampValidator</type>
      <name>timestamp_validator_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>aa86ddae38f360ec1b71523b121b862ed</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>mode_status_timer_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>ab19dbe87f70033ab86467374620bed85</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Subscription&lt; ros2_uav_interfaces::msg::Disturbance &gt;::SharedPtr</type>
      <name>disturbance_sub_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a5b1287e74568b72963285b3566f65b83</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; px4_ros2::OdometryLocalPosition &gt;</type>
      <name>vehicle_local_position_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>af0e561d7af2a718c26971ca16cd459bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; px4_ros2::OdometryAngularVelocity &gt;</type>
      <name>vehicle_angular_velocity_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a1b7fae8b6ad3c3924e56221e61248921</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; px4_ros2::OdometryAttitude &gt;</type>
      <name>vehicle_attitude_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>a482fbd9034a8b1ae598413ad1cb5571c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::modes::NlmpcPosition</name>
    <filename>classros2__uav_1_1modes_1_1NlmpcPosition.html</filename>
    <base>ros2_uav::modes::RatesThrustMode&lt; uav_cpp::pipelines::NlmpcPosition &gt;</base>
    <member kind="function">
      <type></type>
      <name>NlmpcPosition</name>
      <anchorfile>classros2__uav_1_1modes_1_1NlmpcPosition.html</anchorfile>
      <anchor>ae661e4d02ec1e21f4f8a53ca7fcd76d6</anchor>
      <arglist>(rclcpp::Node &amp;node)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::modes::NlmpcWaypoints</name>
    <filename>classros2__uav_1_1modes_1_1NlmpcWaypoints.html</filename>
    <base>ros2_uav::modes::RatesThrustMode&lt; uav_cpp::pipelines::NlmpcWaypoints &gt;</base>
    <member kind="function">
      <type></type>
      <name>NlmpcWaypoints</name>
      <anchorfile>classros2__uav_1_1modes_1_1NlmpcWaypoints.html</anchorfile>
      <anchor>a4e4a90da12be4adcd12ec202424f2875</anchor>
      <arglist>(rclcpp::Node &amp;node)</arglist>
    </member>
  </compound>
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
    <name>ros2_uav::modes::Position</name>
    <filename>classros2__uav_1_1modes_1_1Position.html</filename>
    <base>ros2_uav::modes::AttitudeThrustMode&lt; uav_cpp::pipelines::Se3Position &gt;</base>
    <member kind="function">
      <type></type>
      <name>Position</name>
      <anchorfile>classros2__uav_1_1modes_1_1Position.html</anchorfile>
      <anchor>ad4ddc4dc2f6de8b2fd9f15015338c2c4</anchor>
      <arglist>(rclcpp::Node &amp;node)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::modes::RatesThrustMode</name>
    <filename>classros2__uav_1_1modes_1_1RatesThrustMode.html</filename>
    <templarg>DerivedFromRatesThrustMode ModeT</templarg>
    <base>ros2_uav::modes::ModeInterface&lt; ModeT &gt;</base>
    <member kind="function">
      <type></type>
      <name>RatesThrustMode</name>
      <anchorfile>classros2__uav_1_1modes_1_1RatesThrustMode.html</anchorfile>
      <anchor>af179021df07a41df773c83389b992473</anchor>
      <arglist>(const ModeBase::Settings &amp;mode_settings, rclcpp::Node &amp;node)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::modes::Spin</name>
    <filename>classros2__uav_1_1modes_1_1Spin.html</filename>
    <base>ros2_uav::modes::AttitudeThrustMode&lt; uav_cpp::pipelines::Spin &gt;</base>
    <member kind="function">
      <type></type>
      <name>Spin</name>
      <anchorfile>classros2__uav_1_1modes_1_1Spin.html</anchorfile>
      <anchor>abf405c412377e80283738fd23d4167e7</anchor>
      <arglist>(rclcpp::Node &amp;node)</arglist>
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
  <compound kind="concept">
    <name>ros2_uav::modes::DerivedFromAttitudeThrustMode</name>
    <filename>conceptros2__uav_1_1modes_1_1DerivedFromAttitudeThrustMode.html</filename>
  </compound>
  <compound kind="concept">
    <name>ros2_uav::modes::DerivedFromRatesThrustMode</name>
    <filename>conceptros2__uav_1_1modes_1_1DerivedFromRatesThrustMode.html</filename>
  </compound>
  <compound kind="concept">
    <name>ros2_uav::modes::DerivedFromUavCppPipeline</name>
    <filename>conceptros2__uav_1_1modes_1_1DerivedFromUavCppPipeline.html</filename>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav</name>
    <filename>namespaceros2__uav.html</filename>
    <namespace>ros2_uav::debug</namespace>
    <namespace>ros2_uav::executors</namespace>
    <namespace>ros2_uav::identification</namespace>
    <namespace>ros2_uav::modes</namespace>
    <namespace>ros2_uav::utils</namespace>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::debug</name>
    <filename>namespaceros2__uav_1_1debug.html</filename>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::executors</name>
    <filename>namespaceros2__uav_1_1executors.html</filename>
    <class kind="class">ros2_uav::executors::ExecutorArm</class>
    <class kind="class">ros2_uav::executors::ExecutorLand</class>
    <class kind="class">ros2_uav::executors::ExecutorTakeOff</class>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::identification</name>
    <filename>namespaceros2__uav_1_1identification.html</filename>
    <class kind="class">ros2_uav::identification::ThrustMatcher</class>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::modes</name>
    <filename>namespaceros2__uav_1_1modes.html</filename>
    <class kind="class">ros2_uav::modes::AttitudeThrustMode</class>
    <class kind="class">ros2_uav::modes::ModeInterface</class>
    <class kind="class">ros2_uav::modes::NlmpcPosition</class>
    <class kind="class">ros2_uav::modes::NlmpcWaypoints</class>
    <class kind="class">ros2_uav::modes::Position</class>
    <class kind="class">ros2_uav::modes::RatesThrustMode</class>
    <class kind="class">ros2_uav::modes::Spin</class>
    <concept>ros2_uav::modes::DerivedFromUavCppPipeline</concept>
    <concept>ros2_uav::modes::DerivedFromAttitudeThrustMode</concept>
    <concept>ros2_uav::modes::DerivedFromRatesThrustMode</concept>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::utils</name>
    <filename>namespaceros2__uav_1_1utils.html</filename>
    <class kind="class">ros2_uav::utils::ButterworthFilter</class>
    <class kind="class">ros2_uav::utils::DerivativeFilter</class>
    <class kind="class">ros2_uav::utils::DerivativeFilter3D</class>
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

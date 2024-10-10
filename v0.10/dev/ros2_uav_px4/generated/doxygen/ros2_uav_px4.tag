<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.11.0" doxygen_gitid="9b424b03c9833626cd435af22a444888fbbb192d">
  <compound kind="class">
    <name>ros2_uav::identification::AttitudeThrustMatcher</name>
    <filename>classros2__uav_1_1identification_1_1AttitudeThrustMatcher.html</filename>
    <member kind="function">
      <type>void</type>
      <name>actuatorMotorsCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1AttitudeThrustMatcher.html</anchorfile>
      <anchor>acc2bfcfbb3c61ac6f574d1cd0ecae51a</anchor>
      <arglist>(const ActuatorMotors::SharedPtr actuator_motors)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>odometryCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1AttitudeThrustMatcher.html</anchorfile>
      <anchor>a817bc847554b1308a499ce241d79b09a</anchor>
      <arglist>(const VehicleOdometry::SharedPtr odometry)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>accelerationCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1AttitudeThrustMatcher.html</anchorfile>
      <anchor>a16789e2db948ccf0b6b5e206b1fc515f</anchor>
      <arglist>(const VehicleAcceleration::SharedPtr acceleration)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>controlModeCallback</name>
      <anchorfile>classros2__uav_1_1identification_1_1AttitudeThrustMatcher.html</anchorfile>
      <anchor>ae2f492fea952d30570fc796f74d52438</anchor>
      <arglist>(const VehicleControlMode::SharedPtr control_mode)</arglist>
    </member>
  </compound>
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
    <name>uav_ros2::utils::ButterworthFilter</name>
    <filename>classuav__ros2_1_1utils_1_1ButterworthFilter.html</filename>
    <member kind="function">
      <type></type>
      <name>ButterworthFilter</name>
      <anchorfile>classuav__ros2_1_1utils_1_1ButterworthFilter.html</anchorfile>
      <anchor>a59d03b3053292552878e0c8c4aef4f5d</anchor>
      <arglist>(double cutoff_frequency, double sampling_frequency)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>filtfilt</name>
      <anchorfile>classuav__ros2_1_1utils_1_1ButterworthFilter.html</anchorfile>
      <anchor>a54b18a483449bcca6c8f6b2825f47ec5</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;input)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>filter</name>
      <anchorfile>classuav__ros2_1_1utils_1_1ButterworthFilter.html</anchorfile>
      <anchor>afe7c19fecc39baf413f6fe09374871b9</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;input)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>uav_ros2::utils::DataLogger</name>
    <filename>classuav__ros2_1_1utils_1_1DataLogger.html</filename>
  </compound>
  <compound kind="class">
    <name>uav_ros2::utils::DerivativeFilter</name>
    <filename>classuav__ros2_1_1utils_1_1DerivativeFilter.html</filename>
    <member kind="function">
      <type></type>
      <name>DerivativeFilter</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>ab7eb4d62ea2a34104272b2180a45fae2</anchor>
      <arglist>(double cutoff_frequency, double sampling_frequency)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addData</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>ae721839f157305920e88bb73b1e7f3a2</anchor>
      <arglist>(double value)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>computeDerivative</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>af07f8bfac510099c09d35af9b86a1f88</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>process</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>ac658dc137ecb4db7c04d44085a0cc561</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>process</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter.html</anchorfile>
      <anchor>a74634f34d390f523b578a5b6a1295713</anchor>
      <arglist>(std::vector&lt; double &gt; &amp;derivative, std::vector&lt; double &gt; &amp;derivative_filtered)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>uav_ros2::utils::DerivativeFilter3D</name>
    <filename>classuav__ros2_1_1utils_1_1DerivativeFilter3D.html</filename>
    <member kind="function">
      <type></type>
      <name>DerivativeFilter3D</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>a0b1f8cba1d477af9e1f4cb776f666c5b</anchor>
      <arglist>(double cutoff_frequency, double sampling_frequency)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addData</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>aa6370180c60ff52aa362a23cc5661f7e</anchor>
      <arglist>(const tf2::Vector3 &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; tf2::Vector3 &gt;</type>
      <name>process</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>ac7e2254b5b7e9338ebd648c691c0ad30</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>process</name>
      <anchorfile>classuav__ros2_1_1utils_1_1DerivativeFilter3D.html</anchorfile>
      <anchor>acdfa8e5048a1a85f5d4e12f4120bc6e6</anchor>
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
      <anchor>ab0509fad34e57925fe2ca36477e33d60</anchor>
      <arglist>(rclcpp::Node &amp;node, px4_ros2::ModeBase &amp;owned_mode)</arglist>
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
      <type>void</type>
      <name>setTfBuffer</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>ace6dbff28f09bbeb4114e53c05d9edf5</anchor>
      <arglist>(std::shared_ptr&lt; tf2_ros::Buffer &gt; tf_buffer)</arglist>
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
    <name>ros2_uav::executors</name>
    <filename>namespaceros2__uav_1_1executors.html</filename>
    <class kind="class">ros2_uav::executors::ExecutorArm</class>
    <class kind="class">ros2_uav::executors::ExecutorTakeOff</class>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::identification</name>
    <filename>namespaceros2__uav_1_1identification.html</filename>
    <class kind="class">ros2_uav::identification::AttitudeThrustMatcher</class>
  </compound>
  <compound kind="namespace">
    <name>ros2_uav::modes</name>
    <filename>namespaceros2__uav_1_1modes.html</filename>
    <class kind="class">ros2_uav::modes::AttitudeThrustMode</class>
    <class kind="class">ros2_uav::modes::ModeInterface</class>
    <class kind="class">ros2_uav::modes::NlmpcPosition</class>
    <class kind="class">ros2_uav::modes::Position</class>
    <class kind="class">ros2_uav::modes::RatesThrustMode</class>
    <class kind="class">ros2_uav::modes::Spin</class>
    <concept>ros2_uav::modes::DerivedFromUavCppPipeline</concept>
    <concept>ros2_uav::modes::DerivedFromAttitudeThrustMode</concept>
    <concept>ros2_uav::modes::DerivedFromRatesThrustMode</concept>
  </compound>
  <compound kind="namespace">
    <name>uav_ros2::debug</name>
    <filename>namespaceuav__ros2_1_1debug.html</filename>
  </compound>
  <compound kind="namespace">
    <name>uav_ros2::utils</name>
    <filename>namespaceuav__ros2_1_1utils.html</filename>
    <class kind="class">uav_ros2::utils::ButterworthFilter</class>
    <class kind="class">uav_ros2::utils::DataLogger</class>
    <class kind="class">uav_ros2::utils::DerivativeFilter</class>
    <class kind="class">uav_ros2::utils::DerivativeFilter3D</class>
    <member kind="function">
      <type>tf2::Vector3</type>
      <name>eigenNedToTf2Nwu</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>a7614bb5189983a1cbd6a1b7e01526c5e</anchor>
      <arglist>(const Eigen::Vector3f &amp;eigen_vector)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Vector3f</type>
      <name>tf2FwuToEigenNed</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>ae85fdca6c571963afa4a14f44bc3da3a</anchor>
      <arglist>(const tf2::Vector3 &amp;tf2_vector)</arglist>
    </member>
    <member kind="function">
      <type>tf2::Quaternion</type>
      <name>eigenNedToTf2Nwu</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>aebe982ad1e77aed280740bd6114c8380</anchor>
      <arglist>(const Eigen::Quaternionf &amp;eigen_quaternion)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Quaternionf</type>
      <name>tf2FwuToEigenNed</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>ac168240a86c1b5aec05b18a6009e96eb</anchor>
      <arglist>(const tf2::Quaternion &amp;tf2_quaternion)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::pipelines::PoseHeading</type>
      <name>convertFromRosMsg</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>add5097611370add723120f1fc5a15269</anchor>
      <arglist>(const ros2_uav_interfaces::msg::PoseHeading &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::pipelines::AttitudeThrust</type>
      <name>convertFromPx4Msg</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>a7316389ae8eeb5e4a08251842a549e6c</anchor>
      <arglist>(const px4_msgs::msg::VehicleAttitudeSetpoint &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::pipelines::Odometry</type>
      <name>convertFromPx4Msg</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>ab9fa271a9b8caba3469db163b811ffcf</anchor>
      <arglist>(const px4_msgs::msg::VehicleOdometry &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>uav_cpp::pipelines::Thrust</type>
      <name>convertFromPx4Msg</name>
      <anchorfile>namespaceuav__ros2_1_1utils.html</anchorfile>
      <anchor>a3ecedfcff05f08ddf7bee36954c33b7b</anchor>
      <arglist>(const px4_msgs::msg::VehicleThrustSetpoint &amp;msg)</arglist>
    </member>
  </compound>
</tagfile>

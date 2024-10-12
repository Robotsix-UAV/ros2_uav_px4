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
      <type>void</type>
      <name>onActivate</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorArm.html</anchorfile>
      <anchor>a55f72c653f1e62a2abdb770a589beaa3</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>onDeactivate</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorArm.html</anchorfile>
      <anchor>ab1b1330a891d62aba05e795ac3300ac4</anchor>
      <arglist>(DeactivateReason reason) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>runState</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorArm.html</anchorfile>
      <anchor>aa392e6b97a26a1e2bf36469a63d147e5</anchor>
      <arglist>(State state)</arglist>
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
      <type>void</type>
      <name>onActivate</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorTakeOff.html</anchorfile>
      <anchor>a7943df4bccf2bc8d6f91a015341e92db</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>onDeactivate</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorTakeOff.html</anchorfile>
      <anchor>adf6e5bd91e2079a5a9755b201aec77f9</anchor>
      <arglist>(DeactivateReason reason) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>runState</name>
      <anchorfile>classros2__uav_1_1executors_1_1ExecutorTakeOff.html</anchorfile>
      <anchor>a0df3ec05fc67089936f4556e244d1c7e</anchor>
      <arglist>(State state)</arglist>
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
      <anchor>a9c071e6c5bbfb0f68ef292e001121023</anchor>
      <arglist>(const PipelineT::InputType &amp;setpoint)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTfBuffer</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>ace6dbff28f09bbeb4114e53c05d9edf5</anchor>
      <arglist>(std::shared_ptr&lt; tf2_ros::Buffer &gt; tf_buffer)</arglist>
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
      <type>rclcpp::Publisher&lt; ros2_uav_interfaces::msg::Coordinate &gt;::SharedPtr</type>
      <name>coordinate_publisher_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>ade41f0980a8737296098406648c5fa55</anchor>
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
    <name>ros2_uav::modes::DerivedFromUavCppPipeline</name>
    <filename>conceptros2__uav_1_1modes_1_1DerivedFromUavCppPipeline.html</filename>
  </compound>
</tagfile>

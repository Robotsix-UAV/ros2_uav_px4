<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.1">
  <compound kind="class">
    <name>ros2_uav::modes::AttitudeThrustMode</name>
    <filename>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</filename>
    <templarg>ModeT</templarg>
    <base>ros2_uav::modes::ModeInterface</base>
    <member kind="function">
      <type></type>
      <name>AttitudeThrustMode</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>aed228c48e33e8302942ea1dbfd1115a7</anchor>
      <arglist>(const ModeBase::Settings &amp;mode_settings, rclcpp::Node &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setSetpoint</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>ab58ade10b89052e9b92a2b7622909a78</anchor>
      <arglist>(const ModeT::TrackerType::SetPointType &amp;setpoint)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTfBuffer</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>ace778bebbd6ffb26217129a4d1b1cef4</anchor>
      <arglist>(std::shared_ptr&lt; tf2_ros::Buffer &gt; tf_buffer)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AttitudeThrustMode&lt; uav_cpp::modes::Se3Position &gt;</name>
    <filename>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</filename>
    <base>ros2_uav::modes::ModeInterface</base>
    <member kind="function">
      <type></type>
      <name>AttitudeThrustMode</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>aed228c48e33e8302942ea1dbfd1115a7</anchor>
      <arglist>(const ModeBase::Settings &amp;mode_settings, rclcpp::Node &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setSetpoint</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>ab58ade10b89052e9b92a2b7622909a78</anchor>
      <arglist>(const ModeT::TrackerType::SetPointType &amp;setpoint)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTfBuffer</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>ace778bebbd6ffb26217129a4d1b1cef4</anchor>
      <arglist>(std::shared_ptr&lt; tf2_ros::Buffer &gt; tf_buffer)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AttitudeThrustMode&lt; uav_cpp::modes::Spin &gt;</name>
    <filename>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</filename>
    <base>ros2_uav::modes::ModeInterface</base>
    <member kind="function">
      <type></type>
      <name>AttitudeThrustMode</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>aed228c48e33e8302942ea1dbfd1115a7</anchor>
      <arglist>(const ModeBase::Settings &amp;mode_settings, rclcpp::Node &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setSetpoint</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>ab58ade10b89052e9b92a2b7622909a78</anchor>
      <arglist>(const ModeT::TrackerType::SetPointType &amp;setpoint)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTfBuffer</name>
      <anchorfile>classros2__uav_1_1modes_1_1AttitudeThrustMode.html</anchorfile>
      <anchor>ace778bebbd6ffb26217129a4d1b1cef4</anchor>
      <arglist>(std::shared_ptr&lt; tf2_ros::Buffer &gt; tf_buffer)</arglist>
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
      <anchor>a942855c3a4b2c8978344c3858ddbb489</anchor>
      <arglist>([[maybe_unused]] DeactivateReason reason) override</arglist>
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
      <anchor>ad506c9211cc3d58d4f8965049debc9a7</anchor>
      <arglist>([[maybe_unused]] DeactivateReason reason) override</arglist>
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
    <member kind="function">
      <type></type>
      <name>ModeInterface</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>af94a63c00f06dd58e5a128b1127d5fe6</anchor>
      <arglist>(const ModeBase::Settings &amp;mode_settings, rclcpp::Node &amp;node)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Node &amp;</type>
      <name>node_</name>
      <anchorfile>classros2__uav_1_1modes_1_1ModeInterface.html</anchorfile>
      <anchor>ac31aefe09b2b92e6345909805f718b0f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ros2_uav::modes::Position</name>
    <filename>classros2__uav_1_1modes_1_1Position.html</filename>
    <base>AttitudeThrustMode&lt; uav_cpp::modes::Se3Position &gt;</base>
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
    <base>AttitudeThrustMode&lt; uav_cpp::modes::Spin &gt;</base>
    <member kind="function">
      <type></type>
      <name>Spin</name>
      <anchorfile>classros2__uav_1_1modes_1_1Spin.html</anchorfile>
      <anchor>abf405c412377e80283738fd23d4167e7</anchor>
      <arglist>(rclcpp::Node &amp;node)</arglist>
    </member>
  </compound>
</tagfile>

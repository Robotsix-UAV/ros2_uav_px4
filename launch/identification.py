from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define a launch argument for the namespace
    uav_namespace_arg = DeclareLaunchArgument(
        'uav_namespace',
        default_value='/uav0',
        description='Namespace for the UAV'
    )

    # Node for thrust matcher
    thrust_matcher_node = Node(
        package='ros2_uav_px4',
        executable='thrust_matcher',
        namespace=LaunchConfiguration('uav_namespace'),
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        uav_namespace_arg,
        thrust_matcher_node
    ])

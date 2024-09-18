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

    # Node for the parameter server
    parameter_server_node = Node(
        package='ros2_uav_parameters',
        executable='parameter_server',
        namespace=LaunchConfiguration('uav_namespace'),
        output='screen',
        emulate_tty=True
    )

    # Node for modes runner
    modes_runner_node = Node(
        package='ros2_uav_px4',
        executable='modes_runner',
        namespace=LaunchConfiguration('uav_namespace'),
        output='screen',
        emulate_tty=True
    )

    # Node for the disturbance observer
    disturbance_observer_node = Node(
        package='ros2_uav_px4',
        executable='disturbance_observer',
        namespace=LaunchConfiguration('uav_namespace'),
        output='screen',
        emulate_tty=True
    )

    # Node for tf publisher
    tf_publisher_node = Node(
        package='ros2_uav_px4',
        executable='tf_publisher',
        namespace=LaunchConfiguration('uav_namespace'),
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        uav_namespace_arg,
        parameter_server_node,
        modes_runner_node,
        disturbance_observer_node,
        tf_publisher_node
    ])

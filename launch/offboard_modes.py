# Copyright 2024 The Technology Innovation Institute (TII)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Damien Six (damien@robotsix.net)

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
    controller_node = Node(
        package='ros2_uav_px4',
        executable='controller',
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

    return LaunchDescription([
        uav_namespace_arg,
        parameter_server_node,
        controller_node,
        disturbance_observer_node
    ])

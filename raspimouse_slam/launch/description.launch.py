# Copyright 2022 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Declare arguments #
    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='none',
        description='Set "none", "urg", "lds", or "rplidar".',
    )

    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame', default_value='laser', description='Set lidar link name.'
    )

    declare_arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Set namespace for tf tree.'
    )

    xacro_file = os.path.join(
        get_package_share_directory('raspimouse_description'),
        'urdf',
        'raspimouse.urdf.xacro',
    )

    params = {
        'robot_description': Command(
            [
                'xacro ',
                xacro_file,
                ' lidar:=',
                LaunchConfiguration('lidar'),
                ' lidar_frame:=',
                LaunchConfiguration('lidar_frame'),
            ]
        ),
        'frame_prefix': [LaunchConfiguration('namespace'), '/'],
    }

    push_ns = PushRosNamespace([LaunchConfiguration('namespace')])

    # Nodes #
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_arg_lidar)
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_namespace)

    ld.add_action(push_ns)

    ld.add_action(robot_state_pub_node)
    ld.add_action(joint_state_pub_node)
    return ld

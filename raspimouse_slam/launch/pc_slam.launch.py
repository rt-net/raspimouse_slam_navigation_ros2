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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments #
    slam_config_file = LaunchConfiguration('slam_config_file')
    rviz2_file = LaunchConfiguration('rviz2_file')

    declare_arg_slam_config_file = DeclareLaunchArgument(
        'slam_config_file',
        default_value=os.path.join(
            get_package_share_directory('raspimouse_slam'),
            'config',
            'mapper_params_online_sync.yaml'),
        description='The full path to the config file for SLAM')

    declare_arg_rviz2_config_path = DeclareLaunchArgument(
        'rviz2_file', default_value=os.path.join(
            get_package_share_directory('raspimouse_slam'),
            'rviz',
            'default.rviz'),
        description='The full path to the rviz file'
    )

    # Nodes #
    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[slam_config_file],
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz2_file],
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_slam_config_file)
    ld.add_action(declare_arg_rviz2_config_path)

    ld.add_action(slam_node)
    ld.add_action(rviz2_node)

    return ld

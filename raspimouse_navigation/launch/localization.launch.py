# Copyright 2026 RT Corporation
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import Parameter, ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)
    use_sim_time = Parameter(
        'use_sim_time', LaunchConfiguration('use_sim_time'), value_type=bool
    )
    parameters = [params_file, use_sim_time]

    return LaunchDescription([
        DeclareLaunchArgument('map', description='The full path to the map yaml file.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('raspimouse_navigation'), 'params', 'raspimouse.yaml'
            ]),
            description='The full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=parameters + [{
                'yaml_filename': ParameterValue(LaunchConfiguration('map'), value_type=str),
            }],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen', parameters=parameters,
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=parameters + [{
                'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])

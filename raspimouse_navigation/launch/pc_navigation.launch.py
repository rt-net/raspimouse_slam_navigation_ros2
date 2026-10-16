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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import Parameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('raspimouse_navigation')

    declare_arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='True: Simulate, False: Machine',
    )

    declare_arg_map = DeclareLaunchArgument(
        'map', description='The full path to the map yaml file.'
    )

    declare_arg_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [package_share, 'params', 'raspimouse.yaml']
        ),
        description='The full path to the param file.',
    )

    declare_arg_rviz2_config_path = DeclareLaunchArgument(
        'rviz2_file',
        default_value=PathJoinSubstitution(
            [package_share, 'rviz', 'nav2_view.rviz']
        ),
        description='The full path to the rviz file',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz2_file = LaunchConfiguration('rviz2_file')

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
            )
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            # This robot does not provide keepout or speed-zone mask maps.
            'use_keepout_zones': 'false',
            'use_speed_zones': 'false',
        }.items(),
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz2_file],
        parameters=[Parameter('use_sim_time', use_sim_time, value_type=bool)],
    )

    return LaunchDescription(
        [
            declare_arg_use_sim_time,
            declare_arg_map,
            declare_arg_params_file,
            declare_arg_rviz2_config_path,
            nav2_node,
            rviz2_node,
        ]
    )

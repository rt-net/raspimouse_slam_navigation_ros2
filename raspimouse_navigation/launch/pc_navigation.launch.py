# Copyright 2021 RT Corporation
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

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_robot = DeclareLaunchArgument(
        'robot', default_value='raspimouse',
        description='The name of the robot.'
    )

    # map_dir = DeclareLaunchArgument(
    #    'map_file', default_value=os.path.join(
    map_dir = LaunchConfiguration(
        'map_file', default=os.path.join(
            get_package_share_directory('raspimouse_slam'),
            'maps',
            'test.yaml'),
    #    description='The path to the map file.'
    )
    
    # param_dir = DeclareLaunchArgument
    #    'param_file', default_value=os.path.join(
    param_dir = LaunchConfiguration(
        'param_file', default=os.path.join(
            get_package_share_directory('raspimouse_navigation'),
            'param',
            'raspimouse.yaml'),
    #    description='The path to the param file.'
    )

    rviz2_config_dir = LaunchConfiguration(
        'rviz_file', default=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz',
            'nav2_default_view.rviz')
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': 'map_dir',
            'params_file': LaunchConfiguration[param_dir],
            'use_sim_time': use_sim_time}.items(),
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2', executable='rviz2', output='screen',
        arguments=[
            '-d', rviz2_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
    )


    ld = LaunchDescription()
    ld.add_action(declare_robot)
    ld.add_action(map_dir)
    ld.add_action(param_dir)
    #ld.add_action(rviz2_config_dir)

    ld.add_action(nav2_node)
    ld.add_action(rviz2_node)

    return ld

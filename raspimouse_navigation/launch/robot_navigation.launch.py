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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file = LaunchConfiguration('xacro_file')

    declare_lidar = DeclareLaunchArgument(
        'lidar', default_value='lds',
        description='LiDAR: lds only, for now.'
    )

    xacro_path = DeclareLaunchArgument(
        'xacro_file', default_value=os.path.join(
            get_package_share_directory('raspimouse_description'),
            'urdf',
            'raspimouse.urdf.xacro'),
        description='Path to xacro file.'
    )

    mouse_node = LifecycleNode(
        name='raspimouse',
        package='raspimouse', executable='raspimouse', output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'raspimouse_ros2_examples'), 'config', 'mouse.yml')]
    )

    def func_launch_lidar_node(context):
        if context.launch_configurations['lidar'] == 'lds':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('hls_lfcd_lds_driver'),
                    'launch'),
                    '/hlds_laser.launch.py'
                    ]),)]
    launch_lidar_node = OpaqueFunction(function=func_launch_lidar_node)
    
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0.1', '0', '3.14',
            '3.14', 'base_footprint', 'laser'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
        'robot_description': Command(['xacro ', xacro_file])}]
    )


    ld = LaunchDescription()
    ld.add_action(declare_lidar)
    ld.add_action(xacro_path)

    ld.add_action(mouse_node)
    ld.add_action(launch_lidar_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(robot_state_publisher)

    return ld
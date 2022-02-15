# Copyright 2020 RT Corporation
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    lidar_port = LaunchConfiguration(
        'lidar_port', default='/dev/ttyACM0')

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar', default_value='lds',
        description='LiDAR: LDS or URG only, for now.')

    declare_arg_use_lds = DeclareLaunchArgument(
        'use_lds',
        default_value='false',
        description='Set "true" when using lds.')

    declare_arg_use_urg = DeclareLaunchArgument(
        'use_urg',
        default_value='false',
        description='Set "true" when using urg.')
    
    mouse_node = LifecycleNode(
        name='raspimouse',
        package='raspimouse', executable='raspimouse', output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'raspimouse_slam'), 'config', 'mouse.yaml')]
    )

    lds_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('hls_lfcd_lds_driver'),
                'launch'),
                '/hlds_laser.launch.py']),
        condition=IfCondition(LaunchConfiguration('use_lds'))
    )

    urg_launch = Node(
        name='urg_node_driver',
        package='urg_node', executable='urg_node_driver', output='screen',
        parameters=[{'serial_port': lidar_port}],
        condition=IfCondition(LaunchConfiguration('use_urg'))
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_lidar)
    ld.add_action(declare_arg_use_lds)
    ld.add_action(declare_arg_use_urg)

    ld.add_action(mouse_node)
    ld.add_action(lds_launch)
    ld.add_action(urg_launch)

    return ld
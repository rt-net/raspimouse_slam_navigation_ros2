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

    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='none',
        description='Set "none" or "urg".')

    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='Set lidar link name.')

    declare_arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Set namespace for tf tree.')

    declare_lidar = DeclareLaunchArgument(
        'lidarconfig', default_value='lds',
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
        if context.launch_configurations['lidarconfig'] == 'lds':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('hls_lfcd_lds_driver'),
                    'launch'),
                    '/hlds_laser.launch.py'
                    ]),)]
    launch_lidar_node = OpaqueFunction(function=func_launch_lidar_node)

    params = {'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', xacro_file,
                                            ' lidar:=', LaunchConfiguration('lidar'),
                                            ' lidar_frame:=', LaunchConfiguration('lidar_frame'),
                                            ]),
                'frame_prefix': [LaunchConfiguration('namespace'), '/']}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
        remappings=remappings
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_lidar)
    ld.add_action(declare_arg_lidar)
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_namespace)
    ld.add_action(xacro_path)

    ld.add_action(mouse_node)
    ld.add_action(launch_lidar_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld
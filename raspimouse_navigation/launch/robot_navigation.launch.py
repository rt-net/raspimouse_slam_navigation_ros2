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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file = LaunchConfiguration('xacro_file')
    lidar_port = LaunchConfiguration(
        'lidar_port', default='/dev/ttyACM0')

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='none',
        description='Set "none", "urg", or "lds".')

    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='Set lidar frame name.')

    declare_arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Set namespace for tf tree.')

    declare_arg_use_lds = DeclareLaunchArgument(
        'use_lds',
        default_value='false',
        description='Set "true" when using lds.')

    declare_arg_use_urg = DeclareLaunchArgument(
        'use_urg',
        default_value='false',
        description='Set "true" when using urg.')
    
    declare_arg_xacro_path = DeclareLaunchArgument(
        'xacro_file', default_value=os.path.join(
            get_package_share_directory('raspimouse_description'),
            'urdf',
            'raspimouse.urdf.xacro'),
        description='Path to xacro file.'
    )

    configure_raspimouse_node = ExecuteProcess(
        cmd=[['sleep 5 && ros2 lifecycle set raspimouse configure']],
        shell=True,
        output='screen',
    )

    activate_raspimouse_node = ExecuteProcess(
        cmd=[['ros2 lifecycle set raspimouse activate']],
        shell=True,
        output='screen',
    )

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

    robot_state_params = {'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', xacro_file,
                                            ' lidar:=', LaunchConfiguration('lidar'),
                                            ' lidar_frame:=', LaunchConfiguration('lidar_frame'),
                                            ]),
                'frame_prefix': [LaunchConfiguration('namespace'), '/']}

    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[robot_state_params],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    config_mouse_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=mouse_node,
            on_start=[configure_raspimouse_node],
        )
    )

    active_mouse_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_raspimouse_node,
            on_exit=[activate_raspimouse_node],
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_lidar)
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_namespace)
    ld.add_action(declare_arg_use_lds)
    ld.add_action(declare_arg_use_urg)
    ld.add_action(declare_arg_xacro_path)

    ld.add_action(mouse_node)
    ld.add_action(lds_launch)
    ld.add_action(urg_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(config_mouse_node)
    ld.add_action(active_mouse_node)
    return ld
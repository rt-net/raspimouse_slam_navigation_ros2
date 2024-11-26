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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import LaunchConfigurationEquals
from launch.events import matches_action, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events import lifecycle
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Launch arguments #
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='none',
        description='Set "none", "urg", "lds", or "rplidar".',
    )

    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame', default_value='laser', description='Set lidar frame name.'
    )

    declare_arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Set namespace for tf tree.'
    )

    # Launch files and Nodes #
    lds_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('hls_lfcd_lds_driver'), 'launch'
                ),
                '/hlds_laser.launch.py',
            ]
        ),
        condition=LaunchConfigurationEquals('lidar', 'lds'),
    )

    urg_launch = Node(
        name='urg_node_driver',
        package='urg_node',
        executable='urg_node_driver',
        output='screen',
        parameters=[{'serial_port': lidar_port}],
        condition=LaunchConfigurationEquals('lidar', 'urg'),
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
                '/rplidar_a1_launch.py',
            ]
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'frame_id': LaunchConfiguration('lidar_frame'),
        }.items(),
        condition=LaunchConfigurationEquals('lidar', 'rplidar'),
    )

    description_params = {
        'lidar': LaunchConfiguration('lidar'),
        'lidar_frame': LaunchConfiguration('lidar_frame'),
        'namespace': LaunchConfiguration('namespace'),
    }.items()

    # Launch files and Nodes #
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('raspimouse_slam'), 'launch/'),
                'description.launch.py',
            ]
        ),
        launch_arguments=description_params,
    )

    mouse_node = LifecycleNode(
        name='raspimouse',
        namespace='',
        package='raspimouse',
        executable='raspimouse',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('raspimouse_slam'), 'config', 'mouse.yaml'
            )
        ],
    )

    emit_configuring_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(mouse_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_activating_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(mouse_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    emit_shutdown_event = EmitEvent(event=Shutdown())

    register_activating_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mouse_node,
            goal_state='inactive',
            entities=[emit_activating_event],
        )
    )

    register_shutting_down_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mouse_node,
            goal_state='finalized',
            entities=[emit_shutdown_event],
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_lidar)
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_namespace)

    ld.add_action(mouse_node)
    ld.add_action(lds_launch)
    ld.add_action(urg_launch)
    ld.add_action(rplidar_launch)
    ld.add_action(robot_description_launch)
    ld.add_action(register_activating_transition)
    ld.add_action(register_shutting_down_transition)
    ld.add_action(emit_configuring_event)
    return ld

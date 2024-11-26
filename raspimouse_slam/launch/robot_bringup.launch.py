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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Declare arguments #
    declare_arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Set namespace for tf tree.'
    )

    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='none',
        description='Set "none", "urg", "lds", or "rplidar".',
    )

    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame', default_value='laser', description='Set lidar frame name.'
    )

    declare_arg_joydev = DeclareLaunchArgument(
        'joydev',
        default_value='/dev/input/js0',
        description='Device file for JoyStick Controller',
    )

    declare_arg_joyconfig = DeclareLaunchArgument(
        'joyconfig',
        default_value='f710',
        description='Keyconfig of joystick controllers: \
                     supported: f710, dualshock3, dualshock4',
    )

    # Launch files and Nodes #
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

    display_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('raspimouse_slam'), 'launch/'),
                'description.launch.py',
            ]
        ),
        launch_arguments=description_params,
    )

    teleop_params = {
        'joydev': LaunchConfiguration('joydev'),
        'joyconfig': LaunchConfiguration('joyconfig'),
    }.items()

    teleop_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('raspimouse_slam'), 'launch/'),
                'teleop.launch.py',
            ]
        ),
        launch_arguments=teleop_params,
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_namespace)
    ld.add_action(declare_arg_lidar)
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_joyconfig)
    ld.add_action(declare_arg_joydev)

    ld.add_action(mouse_node)
    ld.add_action(lds_launch)
    ld.add_action(urg_launch)
    ld.add_action(rplidar_launch)
    ld.add_action(display_robot_launch)
    ld.add_action(teleop_joy_launch)

    return ld

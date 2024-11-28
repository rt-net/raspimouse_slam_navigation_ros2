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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Declare arguments #
    declare_arg_joydev = DeclareLaunchArgument(
        'joydev',
        default_value='/dev/input/js0',
        description='Device file for JoyStick Controller',
    )

    declare_arg_joyconfig = DeclareLaunchArgument(
        'joyconfig',
        default_value='f710',
        description='Keyconfig of joystick controllers. \
                     These are the supported controllers and value names: \
                     F710 -> f710 \
                     DUALSHOCK3 -> dualshock3 \
                     DUALSHOCK4 -> dualshock4',
    )

    declare_arg_mouse = DeclareLaunchArgument(
        'mouse', default_value='false', description='Launch raspimouse node'
    )

    joycon_param = [
        os.path.join(get_package_share_directory('raspimouse_ros2_examples')),
        '/config',
        '/joy_',
        LaunchConfiguration('joyconfig'),
        '.yml',
    ]

    # Nodes #
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev': LaunchConfiguration('joydev')}],
    )

    joystick_control_node = Node(
        package='raspimouse_ros2_examples',
        executable='joystick_control.py',
        parameters=[joycon_param],
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
        condition=IfCondition(LaunchConfiguration('mouse')),
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_joydev)
    ld.add_action(declare_arg_joyconfig)
    ld.add_action(declare_arg_mouse)

    ld.add_action(joy_node)
    ld.add_action(joystick_control_node)
    ld.add_action(mouse_node)

    return ld

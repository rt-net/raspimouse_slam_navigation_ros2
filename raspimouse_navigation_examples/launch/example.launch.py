# Copyright 2025 RT Corporation
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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    declare_example_name = DeclareLaunchArgument(
        'example',
        description=('Set an example executable name: '
                     '[waypoint, ...]')
    )
    
    
    example_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='raspimouse_navigation_examples',
        executable=LaunchConfiguration('example'),
        output='screen'
    )

    return LaunchDescription([
        declare_example_name,
        example_node
    ])

    

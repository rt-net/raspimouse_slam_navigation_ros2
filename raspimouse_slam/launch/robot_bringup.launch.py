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
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    declare_arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Set namespace')

    lidar_port = LaunchConfiguration(
        'lidar_port', default='/dev/ttyUSB0')

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar', default_value='rplidar',
        description='LiDAR: RPLIDAR, LDS or URG only, for now.')
    
    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame', default_value='laser',
        description='Set lidar frame name.')

    declare_arg_description_launch_file = DeclareLaunchArgument(
        'description_launch_file', default_value='description.launch.py',
        description='The launch file to publish the robot description')

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
        condition=LaunchConfigurationEquals('lidar', 'lds')
    )

    urg_launch = Node(
        name='urg_node_driver',
        package='urg_node', executable='urg_node_driver', output='screen',
        parameters=[{'serial_port': lidar_port}],
        condition=LaunchConfigurationEquals('lidar', 'urg')
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'launch'),
            '/sllidar_launch.py']),
        launch_arguments={'serial_port': lidar_port,
                    'frame_id': LaunchConfiguration('lidar_frame')}.items(),
        condition=LaunchConfigurationEquals('lidar', 'rplidar')
    )

    description_params = {'lidar': LaunchConfiguration('lidar'),
                          'lidar_frame': LaunchConfiguration('lidar_frame'),
                          'namespace': LaunchConfiguration('namespace')}.items() 

    display_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('raspimouse_slam'),'launch/'),
            LaunchConfiguration('description_launch_file')]),
        launch_arguments=description_params
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_namespace)
    ld.add_action(declare_arg_lidar)
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_description_launch_file)

    ld.add_action(mouse_node)
    ld.add_action(lds_launch)
    ld.add_action(urg_launch)
    ld.add_action(rplidar_launch)
    ld.add_action(display_robot_launch)

    return ld

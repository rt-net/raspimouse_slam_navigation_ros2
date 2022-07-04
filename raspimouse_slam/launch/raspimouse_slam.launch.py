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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_use_lds = DeclareLaunchArgument(
        'use_lds',
        default_value='false',
        description='Set "true" when using lds.')

    declare_use_urg = DeclareLaunchArgument(
        'use_urg',
        default_value='false',
        description='Set "true" when using urg.')

    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='Set lidar frame name.')

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='rplidar',
        description='Set to "urg", "lds", or "rplidar"')

    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[ 
            get_package_share_directory(
                'raspimouse_slam')
            + '/config/mapper_params_offline.yaml'
        ],
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2', executable='rviz2', output='screen',
        arguments=[
            '-d',
            get_package_share_directory('raspimouse_slam')
            + '/rviz/default.rviz'],
    )

    static_tf_lds_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0.01', '0', '3.14', '3.14',
                    'base_footprint', LaunchConfiguration('lidar_frame')],
        # condition=IfCondition(LaunchConfiguration('use_lds'))
        condition=LaunchConfigurationEquals('lidar', 'lds')
    )

    static_tf_urg_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0.01', '0', '0', '0',
                    'base_footprint', LaunchConfiguration('lidar_frame')],
        # condition=IfCondition(LaunchConfiguration('use_urg'))
        condition=LaunchConfigurationEquals('lidar', 'urg')
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_lds)
    ld.add_action(declare_use_urg)
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_lidar)

    ld.add_action(slam_node)
    ld.add_action(rviz2_node)
    ld.add_action(static_tf_lds_node)
    ld.add_action(static_tf_urg_node)

    return ld

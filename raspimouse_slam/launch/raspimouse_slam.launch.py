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
from launch_ros.actions import Node

def generate_launch_description():
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

    ld = LaunchDescription()
    ld.add_action(declare_arg_lidar_frame)
    ld.add_action(declare_arg_lidar)

    ld.add_action(slam_node)
    ld.add_action(rviz2_node)

    return ld

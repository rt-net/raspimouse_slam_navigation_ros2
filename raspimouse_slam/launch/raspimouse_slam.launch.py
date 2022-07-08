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
    declare_arg_slam_config_file = DeclareLaunchArgument(
        'slam_config_file', default='mapper_params_offline.yaml',
        description='The file name of the config file for SLAM')

    slam_config_path = os.path.join(get_package_share_directory('raspimouse_slam'),
                                    'config',
                                    LaunchConfiguration('slam_config_file'))

    declare_arg_rviz_file = DeclareLaunchArgument(
        'rviz_file', default_value='default.rviz',
        description='The file name of the rviz file')
    
    rviz_path = os.path.join(get_package_share_directory('raspimouse_slam'),
                            'rviz',
                            LaunchConfiguration('rviz_file'))

    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[slam_config_path],
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2', executable='rviz2', output='screen',
        arguments=[
            '-d', rviz_path],
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_slam_config_file)
    ld.add_action(declare_arg_rviz_file)

    ld.add_action(slam_node)
    ld.add_action(rviz2_node)

    return ld

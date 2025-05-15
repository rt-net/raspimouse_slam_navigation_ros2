#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2024 RT Corporation
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

import math

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration


def generate_pose(navigator, x: float, y: float, deg: float) -> PoseStamped:
    wp = PoseStamped()
    wp.header.frame_id = 'map'
    wp.header.stamp = navigator.get_clock().now().to_msg()
    wp.pose.position.x = x
    wp.pose.position.y = y
    rad = math.radians(deg)
    wp.pose.orientation.x = 0.0
    wp.pose.orientation.y = 0.0
    wp.pose.orientation.z = math.sin(rad / 2.0)
    wp.pose.orientation.w = math.cos(rad / 2.0)
    return wp


def main():
    rclpy.init()

    nav = BasicNavigator()

    # Initial pose
    initial_pose = generate_pose(navigator=nav, x=0.0, y=0.0, deg=0.0)
    nav.setInitialPose(initial_pose)

    # Wait for the navigation stack to come up
    nav.waitUntilNav2Active()

    # Set goal_1
    goal_poses = []
    goal_pose1 = generate_pose(navigator=nav, x=2.2, y=1.8, deg=0.0)
    goal_poses.append(goal_pose1)
    # Set goal_2
    goal_pose2 = generate_pose(navigator=nav, x=2.8, y=1.0, deg=45.0)
    goal_poses.append(goal_pose2)
    # Set goal_3
    goal_pose3 = generate_pose(navigator=nav, x=2.0, y=0.5, deg=180.0)
    goal_poses.append(goal_pose3)
    # Set goal_4
    goal_pose4 = generate_pose(navigator=nav, x=1.0, y=0.6, deg=180.0)
    goal_poses.append(goal_pose4)

    nav.goThroughPoses(goal_poses)

    i = 0
    while not nav.isTaskComplete():
        # Update timestampe
        initial_pose.header.stamp = nav.get_clock().now().to_msg()

        # Display feedback every 5 cycles
        i = i + 1
        feedback = nav.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(
                        feedback.estimated_time_remaining
                    ).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Cancel navigation if it does not complete within 120 seconds
            if Duration.from_msg(feedback.navigation_time) > Duration(
                seconds=120.0
            ):
                nav.cancelTask()

    # Do something depending on the return code
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    nav.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()

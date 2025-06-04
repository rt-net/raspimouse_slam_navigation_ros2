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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from copy import deepcopy

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    initial_pose.pose.orientation.z = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for the navigation stack to come up
    navigator.waitUntilNav2Active()

    # Set goal_1
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.2
    goal_pose1.pose.position.y = 1.8
    goal_pose1.pose.orientation.w = 0.0
    goal_pose1.pose.orientation.z = 1.0
    goal_poses.append(goal_pose1)

    # Set goal_2
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.8
    goal_pose2.pose.position.y = 1.0
    goal_pose2.pose.orientation.w = 0.707
    goal_pose2.pose.orientation.z = -0.707
    goal_poses.append(goal_pose2)
    
    # Set goal_3
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 2.0
    goal_pose3.pose.position.y = 0.5
    goal_pose3.pose.orientation.w = 0.707
    goal_pose3.pose.orientation.z = -0.707
    goal_poses.append(goal_pose3)

    # Set goal_4
    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = 1.0
    goal_pose4.pose.position.y = 0.6
    goal_pose4.pose.orientation.w = 0.707
    goal_pose4.pose.orientation.z = -0.707
    goal_poses.append(goal_pose4)

    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():

        # Update timestampe
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()

        # Display feedback every 5 cycles
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Cancel navigation if it does not complete within 120 seconds
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raspimouse_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2024-11-28)
------------------
* Support ROS 2 Jazzy (`#13 <https://github.com/rt-net/raspimouse_slam_navigation_ros2/issues/13>`_)
* Update `slam_node` to utilize LifecycleNode in accordance with changes made in `slam_toolbox`
* Contributors: Kazushi Kurasawa, YusukeKato

2.1.0 (2024-03-05)
------------------
* Use rplidar_a1_launch.py because rplidar.launch.py does not exist. (`#8 <https://github.com/rt-net/raspimouse_slam_navigation_ros2/issues/8>`_)
* Contributors: Shota Aoki

2.0.0 (2023-08-04)
------------------
* Humble対応 (`#6 <https://github.com/rt-net/raspimouse_slam_navigation_ros2/issues/6>`_)
* Contributors: Shuhei Kozasa

1.0.0 (2022-07-29)
------------------
* 1m四方のフィールドでSLAMが実施できるようにパラメータ調整
* 使用していないパラメータファイルを削除
* Save the map file to the home directory and use the absolute path to the map file
* Adds raspimouse_description as a dependent pacakgae
* Update launch file to use rplidar
* Adds new param file. Updates launch file to use it as default
* Adds description to launch teleop launch file
* Adds launch file to publish the robot_description
* Update launch file so that it brings up the node for RPLIDAR
* Change tag to exec_depend. The sllidar package is not included in the rosdep list
* Removes static tf node. raspimouse_description has that covered
* Update launch files. Use LaunchConfigurationEquals instead.
* Adds rplidar related arguments and definitions
* Updates joystick controller file name sequence
* Adds lds and urg options
* Adds necessary arguments for lds and urg
* Adds urg_node package as a dependent package
* Adds config file for SLAM
* Creates raspimouse_slam package
* Contributors: Shota Aoki, Shuhei Kozasa

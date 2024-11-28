^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raspimouse_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2024-11-28)
------------------
* Support ROS 2 Jazzy (`#13 <https://github.com/rt-net/raspimouse_slam_navigation_ros2/issues/13>`_)
* Added the setting `enable_stamped_cmd_vel: True` to standardize the velocity to `TwistStamped` from `Twist`
* Modified the `use_sim_time` setting in `collision_monitor` to be `True` during simulations and `False` during real-world operations
* Changed to provide `use_sim_time` from the launch command
* Removed the descriptions for `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml`, changing to automatically load the default settings
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
* DWB:Criticsのパラメータを調整
* AMCL:回転のノイズを調整
* DWB: 走行速度を小さくした
* AMCL: 回転のノイズパラメータを小さくした
* DWBにゴール判定のプラグインを追加
* DWBの走行速度パラメータをラズパイマウスに合わせて調整
* AMCLのパラメータに仮の初期位置・姿勢を設定
* コストマップのパラメータを調整
* 1m四方のフィールドで自己位置推定ができるように、AMCLのパラメータを調整した
* footprintをラズパイマウスとケーブルの形状に合わせて長方形に変更
* Change the raspimouse node activation method
* Save the map file to the home directory and use the absolute path to the map file
* Adds raspimouse_description as a dependent pacakgae
* Update launch file so that it brings up the node for RPLIDAR
* Removes arguments and replaces with LaunchConfigurationEquals
* Adds lifecycle sequence
* Adds joint_state_publisher node
* Add navigation launch file for Robot
* Add navigation launch file for PC
* Adds README for navigation package
* Creates raspimouse_navigation package
* Contributors: Shota Aoki, Shuhei Kozasa

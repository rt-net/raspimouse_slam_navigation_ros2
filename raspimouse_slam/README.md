# raspimouse_slam

[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)パッケージを使用してSLAM（自己位置推定と地図生成）を行うパッケージです。

## Table of Contents

- [raspimouse\_slam](#raspimouse_slam)
  - [Table of Contents](#table-of-contents)
  - [SLAM](#slam)
    - [Setup](#setup)
      - [Using Raspberry Pi Mouse](#using-raspberry-pi-mouse)
      - [Using Gazebo](#using-gazebo)
    - [Teleoperation](#teleoperation)
      - [Using Raspberry Pi Mouse](#using-raspberry-pi-mouse-1)
      - [Using Gazebo](#using-gazebo-1)
    - [Running SLAM](#running-slam)
      - [Using Raspberry Pi Mouse](#using-raspberry-pi-mouse-2)
      - [Using Gazebo](#using-gazebo-2)
    - [Saving Map](#saving-map)
      - [Using Raspberry Pi Mouse](#using-raspberry-pi-mouse-3)
      - [Using Gazebo](#using-gazebo-3)

## SLAM

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2_with_raspimouse_model.png width=500 />

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.gif width=500 />

実機・Gazeboのどちらでも、キーボードまたはジョイスティックでRaspberry Pi Mouseを操作できます。各章で、使用する環境に合わせて「Using Raspberry Pi Mouse」または「Using Gazebo」の手順を実行してください。

下記のコマンドではゲームパッドとしてLogicool Wireless Gamepad F710を使用しています。

> [!NOTE]
> ゲームパッドの操作方法については、[raspimouse_ros2_examples "joystick_control"](https://github.com/rt-net/raspimouse_ros2_examples#joystick_control)を参照してください。

> [!NOTE]
> Raspberry Pi MouseとRemote PCが通信するため、同一ネットワーク上で同じ`ROS_DOMAIN_ID`を指定する必要があります。詳しい設定方法は、[RT Software Tutorials](https://rt-net.github.io/tutorials/raspimouse/ros/samples.html#raspberry-pipcros)のROS 2タブを開いて参照してください。

### Setup

Raspberry Pi Mouseを起動します。使用するLiDARによってコマンドが違います。

#### Using Raspberry Pi Mouse

Raspberry Pi Mouse上で以下のコマンドを実行し、ロボット本体とLiDARを起動します。ジョイスティックの操作ノードも同時に起動します。

```sh
# RPLIDAR A1の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=rplidar lidar_port:=/dev/ttyUSB0 joyconfig:=f710
# LDS-01の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=lds lidar_port:=/dev/ttyUSB0 joyconfig:=f710
# URG-04LX-UG01の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=urg lidar_port:=/dev/ttyACM0 joyconfig:=f710
``` 

#### Using Gazebo

以下のコマンドを実行し、Gazebo上でRaspberry Pi Mouseを起動します。Gazebo上での実行には、[raspimouse_sim](https://github.com/rt-net/raspimouse_sim/tree/jazzy)パッケージのインストールが必要です。

```sh
# RPLIDAR A1の場合
ros2 launch raspimouse_gazebo raspimouse_with_lakehouse.launch.py lidar:=rplidar
# LDS-01の場合
ros2 launch raspimouse_gazebo raspimouse_with_lakehouse.launch.py lidar:=lds
# URG-04LX-UG01の場合
ros2 launch raspimouse_gazebo raspimouse_with_lakehouse.launch.py lidar:=urg
``` 

### Teleoperation

キーボードまたはジョイスティックを選んで操作します。

#### Using Raspberry Pi Mouse

ジョイスティックで操作する場合は、Setupで操作ノードが起動しているため、追加のコマンドは不要です。

キーボードで操作する場合は、Raspberry Pi Mouse上の別のターミナルで次のコマンドを実行します。操作中は、このターミナルにフォーカスを当ててください。

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

#### Using Gazebo

ジョイスティックで操作する場合は、別のターミナルで次のコマンドを実行します。

```sh
ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=f710 mouse:=false
```

キーボードで操作する場合は、別のターミナルで次のコマンドを実行します。シミュレーション時刻を使用するため、`-p use_sim_time:=true`を指定します。操作中は、このターミナルにフォーカスを当ててください。

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true
```

### Running SLAM

SLAMとRVizを起動し、選択したキーボードまたはジョイスティックでRaspberry Pi Mouseを走らせます。RVizで地図が作成されることを確認します。

#### Using Raspberry Pi Mouse

Remote PC上で次のコマンドを実行します。

```sh
ros2 launch raspimouse_slam pc_slam.launch.py
```

#### Using Gazebo

Remote PC上の別のターミナルで次のコマンドを実行します。`use_sim_time:=true`はSLAMとRVizの両方に適用されます。

```sh
ros2 launch raspimouse_slam pc_slam.launch.py use_sim_time:=true
```

### Saving Map

構築した地図をファイルへ保存します。`MAP_NAME`には任意の名前を指定でき、保存すると`MAP_NAME.pgm`と`MAP_NAME.yaml`の2つのファイルが生成されます。

#### Using Raspberry Pi Mouse

SLAMとロボット側のノードを起動したまま、Remote PC上の別のターミナルで次のコマンドを実行します。

```sh
ros2 run nav2_map_server map_saver_cli -f ~/MAP_NAME
```

保存が完了したら、SLAMとキーボードまたはジョイスティックの操作ノードを終了してから、[Navigation](../raspimouse_navigation/README.md#launching-navigation)へ進みます。

#### Using Gazebo

SLAMとシミュレータを起動したまま、Remote PC上の別のターミナルで次のコマンドを実行します。

```sh
ros2 run nav2_map_server map_saver_cli -f ~/MAP_NAME
```

保存が完了したら、SLAMとキーボードまたはジョイスティックの操作ノードを終了してから、[Navigation](../raspimouse_navigation/README.md#launching-navigation)へ進みます。

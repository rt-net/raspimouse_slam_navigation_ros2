# raspimouse_slam

[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)パッケージを使用してSLAM（自己位置推定と地図生成）を行うパッケージです。

## Table of Contents

- [raspimouse\_slam](#raspimouse_slam)
  - [Table of Contents](#table-of-contents)
  - [SLAM](#slam)

## SLAM

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2_with_raspimouse_model.png width=500 />

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.gif width=500 />

Raspberry Pi Mouse上で、次のコマンドを実行します。LiDARを起動し、ゲームパッドでRaspberry Pi Mouseを制御できるようにします。

下記のコマンドではゲームパッドとしてLogicool Wireless Gamepad F710を使用しています。

> [!NOTE]
> ゲームパッドの操作方法については、[raspimouse_ros2_examples "joystick_control"](https://github.com/rt-net/raspimouse_ros2_examples#joystick_control)を参照してください。

### Setup

Raspberry Pi Mouseを起動します。使用するLiDARによってコマンドが違います。

#### Using Raspberry Pi Mouse

以下のコマンドを実行し、Raspberry Pi Mouseを起動します。

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

別のターミナルを立ち上げ、以下のコマンドを実行します。Gazebo上のRaspberry Pi Mouseを操作できるようになります。

```sh
# キーボードで操作する場合
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
# ジョイスティックコントローラで操作する場合
ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=f710 mouse:=false
```

### Running SLAM

Remote PC上で次のコマンドを実行するとSLAMが開始されます。 RVizが立ち上がり、Raspberry Pi Mouseを動かすと地図が構築されていく様子が確認できます。実機とGazeboの両方で同一のコマンドです。

> [!NOTE]
> Raspberry Pi MouseとRemote PCが通信するため、同一ネットワーク上で同じ`ROS_DOMAIN_ID`を指定する必要があります。詳しい設定方法は、[RT Software Tutorials](https://rt-net.github.io/tutorials/raspimouse/ros/samples.html#raspberry-pipcros)のROS 2タブを開いて参照してください。

```sh
ros2 launch raspimouse_slam pc_slam.launch.py
```

構築した地図をファイルへ保存するために、Remote PC 上で次のコマンドを実行します。

```sh
ros2 run nav2_map_server map_saver_cli -f ~/MAP_NAME
```

コマンドを実行すると`MAP_NAME.pgm`と`MAP_NAME.yaml`の2つのファイルが生成されます。

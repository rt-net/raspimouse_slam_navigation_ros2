# raspimouse_slam


[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)パッケージを使用してSLAM（自己位置推定と地図生成）を行うパッケージです。

## Table of Contents

- [raspimouse\_slam](#raspimouse_slam)
  - [Table of Contents](#table-of-contents)
  - [How to Use Examples](#how-to-use-examples)
    - [SLAM](#slam)
        - [Usage](#usage)

## How to Use Examples

### SLAM

[raspimouse_slam_navigation](./raspimouse_slam_navigation)パッケージを使用してSLAM（自己位置推定と地図生成）を行います。

**ここでは、ゲームパッドとしてLogicool Wireless Gamepad F710を使用しています。**

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2_with_raspimouse_model.png width=500 />

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.gif width=500 />

#### Usage

Raspberry Pi Mouse上で、次のコマンドを実行します。LiDARを起動し、ゲームパッドでRaspberry Pi Mouseを制御することができます。
ゲームパッドの操作方法については、[raspimouse_ros2_examplesの"joystick_control"](https://github.com/rt-net/raspimouse_ros2_examples#joystick_control)を参照してください。

```sh
# RPLIDAR A1の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=rplidar lidar_port:=/dev/ttyUSB0 joyconfig:=f710
# LDS-01の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=lds lidar_port:=/dev/ttyUSB0 joyconfig:=f710
# URG-04LX-UG01の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=urg lidar_port:=/dev/ttyACM0 joyconfig:=f710
```

Remote PC上で次のコマンドを実行するとSLAMをが開始されます。 RVizが立ち上がり、Raspberry Pi Mouseを動かすと地図が構築されていく様子が確認できます。

**Raspberry Pi MouseとRemote PCが通信するため、同じネットワーク上で同じ`ROS_DOMAIN_ID`を指定する必要があります。**（詳しい設定方法についてはこちらの[RT Software Tutorials](https://rt-net.github.io/tutorials/raspimouse/ros/samples.html#raspberry-pipcros)のROS 2タブを開いて参照してください。）

```sh
ros2 launch raspimouse_slam pc_slam.launch.py
```

構築した地図をファイルへ保存するために、Remote PC 上で次のコマンドを実行します。

```sh
ros2 run nav2_map_server map_saver_cli -f ~/MAP_NAME
```

コマンドを実行すると`MAP_NAME.pgm`と`MAP_NAME.yaml`の2つのファイルが生成されます。

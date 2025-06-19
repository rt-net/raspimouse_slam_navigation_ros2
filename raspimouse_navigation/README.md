# raspimouse_navigation

[Nav2](https://docs.nav2.org/)パッケージを使用してRaspberry Pi Mouseを自律移動させるパッケージです。

## Table of Contents

- [raspimouse\_navigation](#raspimouse_navigation)
  - [Table of Contents](#table-of-contents)
  - [Navigation](#navigation)
  - [Parameters](#parameters)


## Navigation

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_with_raspimouse_model.png width=500 />

> [!NOTE]
>　サンプルの実行には、Raspberry Pi MouseとRemote PCが同じネットワーク上で同じ`ROS_DOMAIN_ID`を指定している必要があります。

### Setup

#### Using Raspberry Pi Mouse

Raspberry Pi Mouse上で、次のコマンドを実行します。Raspberry Pi MouseのモータとLiDARを制御するためのノードを起動します。

```sh
# RPLIDAR A1の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=rplidar
# LDS-01の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=lds
# URG-04LX-UG01の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=urg lidar_port:=/dev/ttyACM0
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

### Running Navigation

自己位置推定と経路生成用のノードを起動し、RVizを立ち上げます。
引数のmapパラメータには、SLAMで生成した地図（.yamlファイル）を指定してください。


#### Using Raspberry Pi Mouse

Remote PC上で、次のコマンドを実行します。

```sh
ros2 launch raspimouse_navigation pc_navigation.launch.py map:=$HOME/MAP_NAME.yaml
```

#### Using Gazebo

Gazebo上のRaspberry Pi Mouseに対して実行する場合は`use_sim_time:=true`オプションを指定します。

```sh
ros2 launch raspimouse_navigation pc_navigation.launch.py map:=$HOME/MAP_NAME.yaml use_sim_time:=true 
```

無事RVizが起動したら、初期位置・姿勢を合わせます。RVizの画面上部の*2D Pose Estimate*をクリックします。

地図上でRaspberry Pi Mouseが存在すべき尤もらしい位置をクリックし、**そのままホールド**します。

ホールドしながらカーソルを動かし、表示されている矢印の向きをRaspberry Pi Mouseの尤もらしい向きに合わせてからボタンを離します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_setting_initialpose.gif width=500 />

初期位置・姿勢の指示が完了したら、次は目標位置・姿勢を指示します。RVizの画面上部の*Navigation2 Goal*をクリックします。

地図上で、初期位置・姿勢を合わせた時と同様に、地図上をクリックして目標位置を、ホールドしたままカーソルを動かして目標姿勢を指示します。目標姿勢の指示が完了すると、Raspberry Pi Mouseが自律移動を開始します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_setting_goalpose.gif width=500 />

### Stopping the robot

下記画像のようなナビゲーション用のパネルがRViz左下に表示されます。
*Cancel*ボタンを押すと自律移動が中断されます。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_rviz_panel.png width=300 />

Raspberry Pi Mouseを停止させる別の方法として、モータへの電源供給を止める方法もあります。

次のコマンドを実行すると、ソフトウェア側からモータ電源をON / OFFできます。

```sh
# モータ電源をOFFにする
ros2 service call /motor_power std_srvs/srv/SetBool data:\ false
# モータ電源をONにする
ros2 service call /motor_power std_srvs/srv/SetBool data:\ true
```

安全に気をつけながらRaspberry Pi Mouseに搭載されたスイッチを操作してモータ用電源をOFFにします。

## Parameters

- `use_sim_time`
  - Type: `bool`
  - Default: `false`
  - シミュレーション動作時は`true`、実機動作時は`false`を指定します。このパラメータは内部で起動される`Nav2`関連ノードで使用されます。

**[Nav2](https://docs.nav2.org/)のパラメータについては、以下を参照してください。**

- [Nav2：初めてのロボットセットアップガイド](https://docs.nav2.org/setup_guides/index.html)
- [Nav2：パラメータチューニング方法](https://docs.nav2.org/tuning/index.html)
- [Nav2：パラメータ一覧](https://docs.nav2.org/configuration/index.html)

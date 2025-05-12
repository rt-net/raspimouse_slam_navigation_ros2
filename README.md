# raspimouse_slam_navigation

<!-- 本リポジトリのワークフローステータスバッジを添付します（詳細：https://wiki.moon-rabbit.rt-net.jp/670f811056b3aca9041b9aa3）-->
[![industrial_ci](https://github.com/rt-net/raspimouse_slam_navigation_ros2/actions/workflows/industrial_ci.yaml/badge.svg?branch=main)](https://github.com/rt-net/raspimouse_slam_navigation_ros2/actions/workflows/industrial_ci.yaml)

Raspberry Pi MouseでSLAMとナビゲーションを実行するパッケージ郡です。

その他のRaspberry Pi MouseのROS 2サンプル集は[rt-net/raspimouse_ros2_examples](https://github.com/rt-net/raspimouse_ros2_examples)で紹介しています。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_setting_goalpose.gif width=500 />

## Table of Contents

- [raspimouse_slam_navigation](#raspimouse_slam_navigation)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS distributions](#supported-ros-distributions)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Binary Installation](#binary-installation)
    - [Source Build](#source-build)
  - [QuickStart](#quickstart)
  - [How To Use Examples](#how-to-use-examples)
    - [<Sample名>](#sample名)
      - [Usage](#usage)
  - [Packages](#packages)
  - [Topics](#topics)
    - [Subscribed](#subscribed)
    - [Published](#published)
  - [Services](#services)
  - [Actions](#actions)
  - [Parameters](#parameters)
  - [<etc...Lifecycle,Description)>](#etc-lifecycle-description等)
  - [License](#license)
  - [Contributing](#contributing)
  - [Contributors](#contributors)

## Supported ROS distributions

### ROS 2

- [Humble Hawksbill](https://github.com/rt-net/raspimouse2/tree/humble)
- [Jazzy Jalisco](https://github.com/rt-net/raspimouse2/tree/jazzy)

## Requirements

- Raspberry Pi Mouse
  - [Summary](https://rt-net.jp/products/raspberrypimousev3/)
  - [RT Robot Shop](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=4141)
  - Option Parts
    - [Raspberry Pi4用コネクタ](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3776)
    - [マルチLiDARマウント](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)
- Linux OS
  - Ubuntu 24.04 Server
- Device Driver
  - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
- ROS 2
  - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## Installation

### Binary Installation

```sh
sudo apt install ros-$ROS_DISTRO-raspimouse
```

### Source Build

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/

# Clone package
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_slam_navigation_ros2.git

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## QuickStart

SLAMで地図生成を行い、その地図を利用してRaspberry Pi Mouseを自律移動させます。それぞれの詳しい動かし方などについては[SLAM](#slam)、[ナビゲーション](#navigation)をご参照ください。ここでは例として、ゲームパッドのLogicool F710とレーザ測域センサのRPLIDAR A1を使用しています。

### SLAM

```sh
## ロボット側で以下のコマンドを実行
## ゲームパッドの操作方法については、 https://github.com/rt-net/raspimouse_ros2_examples#joystick_control をご参照ください
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=rplidar joyconfig:=f710
## PC側で以下のコマンドを実行
ros2 launch raspimouse_slam pc_slam.launch.py
## 地図ができたら引き続きPC側で実行
## 新しい端末を開いて次のコマンドを実行しましょう
## MAP_NAMEを地図ファイルの名前に置き換えましょう
ros2 run nav2_map_server map_saver_cli -f ~/MAP_NAME
```
地図の保存が行えたら、各種ノードを終了して次に進んでください。

### Navigation

```sh
## ロボット側で以下のコマンドを実行
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=rplidar
## PC側で以下のコマンドを実行
ros2 launch raspimouse_navigation pc_navigation.launch.py map:=$HOME/MAP_NAME.yaml
```
コマンド実行後にRVizが起動します。RViz上で初期位置や目標位置・姿勢を与えるとRaspberry Pi Mouseが動きます。

## How To Use Examples

### raspimouse_slam

[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)パッケージを使用してSLAM（自己位置推定と地図生成）を行うパッケージです。

**ここでは、ゲームパッドとしてLogicool Wireless Gamepad F710を使用しています。**

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2_with_raspimouse_model.png width=500 />

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.gif width=500 />

### Usage
Raspberry Pi Mouse上で、次のコマンドを実行します。LiDARを起動し、ゲームパッドでRaspberry Pi Mouseを制御することができます。
ゲームパッドの操作方法については、[raspimouse_ros2_examplesの"joystick_control"](https://github.com/rt-net/raspimouse_ros2_examples#joystick_control)をご参照ください。

```sh
# RPLIDAR A1の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=rplidar lidar_port:=/dev/ttyUSB0 joyconfig:=f710
# LDS-01の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=lds lidar_port:=/dev/ttyUSB0 joyconfig:=f710
# URG-04LX-UG01の場合
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=urg lidar_port:=/dev/ttyACM0 joyconfig:=f710
```

Remote PC上で次のコマンドを実行して、SLAMを開始します。 RVizが立ち上がり、Raspberry Pi Mouseを動かすと地図が構築されていく様子が確認できます。

**Raspberry Pi MouseとRemote PCが通信するため、同じネットワーク上で同じ`ROS_DOMAIN_ID`を指定する必要があります。**（詳しい設定方法についてはこちらの[RT Software Tutorials](https://rt-net.github.io/tutorials/raspimouse/ros/samples.html#raspberry-pipcros)のROS 2タブを開いてご参照ください。）

```sh
ros2 launch raspimouse_slam pc_slam.launch.py
```

構築した地図をファイルへ保存するために、Remote PC 上で次のコマンドを実行します。

```sh
ros2 run nav2_map_server map_saver_cli -f ~/MAP_NAME
```

コマンドを実行すると`MAP_NAME.pgm`と`MAP_NAME.yaml`の2つのファイルが生成されます。

### raspimouse_navigation

[Nav2](https://github.com/ros-planning/navigation2)パッケージを使用してRaspberry Pi Mouseを自律移動させるパッケージです。
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_with_raspimouse_model.png width=500 />

また、Raspberry Pi MouseとRemote PCが同じネットワーク上で同じ`ROS_DOMAIN_ID`を指定している必要があります。

### Usage

Raspberry Pi Mouse上で、次のコマンドを実行します。Raspberry Pi MouseのモータとLiDARを制御するためのノードを起動しています。

```sh
# RPLIDAR A1の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=rplidar
# LDS-01の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=lds
# URG-04LX-UG01の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=urg lidar_port:=/dev/ttyACM0
```

<br>

Remote PC上で、次のコマンドを実行します。自己位置推定と経路生成用のノードを起動し、RVizを立ち上げます。
引数のmapパラメータには、SLAMで生成した地図（.yamlファイル）を指定してください。

```sh
ros2 launch raspimouse_navigation pc_navigation.launch.py map:=$HOME/MAP_NAME.yaml
```

<br>

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

安全に気をつけながらRaspberry Pi Mouseに搭載されたスイッチを操作してモータ用電源をOFFにします

### raspimouse_navigation_examples

#### waypoint

Raspberry Pi Mouseが、指定したwaypoint（デフォルトでは4点）をもとにナビゲーションします。

**本サンプルのデフォルトのwaypointoは、シミュレーション上のサンプル地図を想定した４点を指定しています。実機動作時は、ナビゲーション対象の環境に合わせたwaypointを指定してください。**

#### Usage

[raspimouse_navigation_examples/waypoint.py](./raspimouse_navigation_examples/raspimouse_navigation_examples/waypoint.py)コード内の初期値や各種waypointに任意の座標・姿勢を設定してください。

##### 初期位置

```python
# Initial pose
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = navigator.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.orientation.w = 1.0
initial_pose.pose.orientation.z = 0.0
navigator.setInitialPose(initial_pose)
```

##### 各waypoint

```python
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
```

[ナビゲーション](#navigation)を実行した状態で、Remote PC上の新規ターミナルで以下のコマンドを実行します。コードを実行すると、指定したwaypointを通る経路でナビゲーションが開始されます。


```bash
ros2 launch raspimouse_navigation_examples example.launch.py example:=waypoint
```

## Packages

- [raspimouse_slam_navigation](./raspimouse_slam_navigation)
  - 本リポジトリ内の各種パッケージのメタ情報を管理します。

- [raspimouse_slam](./raspimouse_slam)
  - SLAMを実行するパッケージです。

- [raspimouse_navigations](./raspimouse_navigation)
  - ナビゲーション用のパッケージです。

- [raspimouse_navigation_examples](./raspimouse_navigation_examples)
  - ナビゲーションのサンプルプログラムパッケージです。

## Parameters

### raspimouse_navigation

- `use_sim_time`
  - Type: `bool`
  - Default: `false`
  - シミュレーション動作時は`true`、実機動作時は`false`を指定します。このパラメータは内部で起動される`Nav2`関連ノードで使用されます。
  
## License

(C) 2022 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。 特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

## Contributing

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md)に従ってください。

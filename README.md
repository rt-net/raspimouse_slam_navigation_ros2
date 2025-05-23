# raspimouse_slam_navigation

<!-- 本リポジトリのワークフローステータスバッジを添付します（詳細：https://wiki.moon-rabbit.rt-net.jp/670f811056b3aca9041b9aa3）-->
[![industrial_ci](https://github.com/rt-net/raspimouse_slam_navigation_ros2/actions/workflows/industrial_ci.yaml/badge.svg?branch=main)](https://github.com/rt-net/raspimouse_slam_navigation_ros2/actions/workflows/industrial_ci.yaml)

Raspberry Pi MouseでSLAMとナビゲーションを実行するパッケージ郡です。

その他のRaspberry Pi MouseのROS 2サンプル集は[rt-net/raspimouse_ros2_examples](https://github.com/rt-net/raspimouse_ros2_examples)で紹介しています。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_setting_goalpose.gif width=500 />

## Table of Contents

- [raspimouse\_slam\_navigation](#raspimouse_slam_navigation)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS distributions](#supported-ros-distributions)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Binary Installation](#binary-installation)
    - [Source Build](#source-build)
  - [QuickStart](#quickstart)
    - [SLAM](#slam)
    - [Navigation](#navigation)
  - [Packages](#packages)
    - [raspimouse_slam](./raspimouse_slam)
    - [raspimouse_navigation](./raspimouse_navigation)
    - [raspimouse_navigation_examples](./raspimouse_navigation_examples)
  - [How To Use Examples](#how-to-use-examples)
    - [raspimouse_slam](./raspimouse_slam/README.md)
      - [SLAM](./raspimouse_slam/README.md#slam)
    - [raspimouse_navigation](./raspimouse_navigation/README.md#how-to-use-examples)
      - [Navigation](./raspimouse_navigation/README.md#navigation)
    - [raspimouse_navigation_examples](./raspimouse_navigation_examples/README.md#how-to-use-examples)
      - [Waypoint Navigation](./raspimouse_navigation_examples/README.md#waypoint-navigation)

  - [Parameters](#parameters)
    - [raspimouse\_navigation](#raspimouse_navigation)
  - [License](#license)
  - [Contributing](#contributing)


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

## Packages

- [raspimouse_slam_navigation](./raspimouse_slam_navigation)
  - 本リポジトリ内の各種パッケージのメタ情報を管理します。
- [raspimouse_slam](./raspimouse_slam)
  - [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)パッケージを使用してSLAM（自己位置推定と地図生成）を行うパッケージです。
- [raspimouse_navigations](./raspimouse_navigation)
  - [Nav2](https://github.com/ros-planning/navigation2)パッケージを使用してRaspberry Pi Mouseを自律移動させるパッケージです。
- [raspimouse_navigation_examples](./raspimouse_navigation_examples)
  - [raspimouse_navigation_examples](./raspimouse_navigation_examples)パッケージを使用したナビゲーションのサンプルプログラムパッケージです。

## How To Use Examples

サンプルプログラムの収載な動作方法は、各パッケージのREADMEで説明しています。

- [raspimouse_slam](./raspimouse_slam/README.md)
  - [SLAM](./raspimouse_slam/README.md#slam)
- [raspimouse_navigation](./raspimouse_navigation/README.md)
  - [Navigation](./raspimouse_navigation/README.md#navigation)
- [raspimouse_navigation_examples](./raspimouse_navigation_examples/README.md)
  - [Waypoint Navigation](./raspimouse_navigation_examples/README.md#waypoint-navigation)
  
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

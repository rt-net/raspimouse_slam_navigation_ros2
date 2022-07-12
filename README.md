# raspimouse_slam_navigation
Raspberry Pi MouseでSLAMやナビゲーションを行うROS 2メタパッケージです。  
その他のRaspberry Pi MouseのROS 2サンプル集は[rt-net/raspimouse_ros2_examples](https://github.com/rt-net/raspimouse_ros2_examples)で紹介しています。  

現在、以下のROS 2のディストリビューションに対応しております。  
 - Foxy ([foxy-devel](TO BE ADDED))

---
# Table of Contents
 - [Requirements](#Requirements)
 - [Installation](#Installation)
 - [QuickStart](#QuickStart)
 - [raspimouse_slam](#SLAM)
 - [raspimouse_navigation](#Navigation)
 - [License](#License)
---

<a name="Requirements"></a>
## Requirements
Some requirements...

<a name="Installation"></a>
## Installation
### Raspberry Pi Mouse V3
以下のコマンドをロボット側で実行してインストールを行います。
```sh
cd ~/ros2_ws/src
# Clone the ROS packages
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_ros2_examples
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_descriptions
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_slam_navigation_ros2
git clone https://github.com/Slamtec/sllidar_ros2
# Install dependencies
rosdep install -r -y --from-paths . --ignore-src

# make and install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Remote PC
以下のコマンドをRemote PC側で実行してインストールを行います。
```sh
cd ~/ros2_ws/src
# Clone the ROS packages
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_slam_navigation_ros2
# Install dependencies
rosdep install -r -y --from-paths . --ignore-src

# make and install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

<a name="QuickStart"></a>
## QuickStart
無事インストールが完了したら、以下の一連のコマンドを実行しましょう。SLAMで地図生成を行い、作った地図を利用してナビゲーションを行うことができます。それぞれの詳しい動かし方などについては[SLAM](#slam)、[ナビゲーション](#navigation)を参照してください。  
ここでは例として、ゲームパッドのLogicool F710とレーザ測域センサのRPLIDAR A1を使用しています。
```sh
# SLAM
## ロボット側で以下のコマンドを実行
## ゲームパッドの操作方法については、 https://github.com/rt-net/raspimouse_ros2_examples#joystick_control を参照してください
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=rplidar lidar_port:=/dev/ttyUSB0 joyconfig:=f710
## PC側で以下のコマンドを実行
ros2 launch raspimouse_slam raspimouse_slam.launch.py
## 地図ができたら引き続きPC側で実行
cd ~/ros2_ws/src/raspimouse_slam_navigation_ros2/raspimouse_slam
mkdir maps && cd maps
ros2 run nav2_map_server map_saver_cli -f $MAP_NAME

# Navigation
## 作ったマップを登録しておく必要があります
cd ~/ros2_ws/
colcon build --symlink-install
source install/setup.bash
## ロボット側で以下のコマンドを実行
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=rplidar
## PC側で以下のコマンドを実行
ros2 launch raspimouse_navigation pc_navigation.launch.py map_file:=$MAP_NAME.yaml
```

<a name="SLAM"></a>
## raspimouse_slam
LiDARをつかってSLAM（自己位置推定と地図生成を行うパッケージです。  
画像？  

ここでは、レーザ測域センサとして**CHANGE TO RPLIDAR A1**、ゲームパッドとして[Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)を使用しています。

### Usage

```sh
some commands
```

### Video


<a name="Navigation"></a>
## raspimouse_navigation

Some explanations...

### Usage

```sh
some commands...
```

### Stopping the robot
また、ロボットが予期しない挙動をした場合は、安全に気をつけながらRaspberry Pi Mouse V3のモータ用電源をOFFにしましょう。 モータ用電源はRaspberry Pi Mouse V3に搭載されたスイッチでON / OFFできます。 あるいは、次のコマンドを実行すると、ソフトウェアスイッチでモータ電源をOFFにできます。与えた目標位置・姿勢への移動を停止したい場合は、新しいターミナルで次のコマンドを実行しましょう。RViz上には目標位置・姿勢が残りますが、ロボットは停止します。新たに、2D Nav Goalを設置すると、そちらに目標位置・姿勢が置き換わります。  
```sh
ros2 service call /motor_power std_srvs//Bool data:\ false\
```  

### Video



<a name="License"></a>
# License

(C) 2022 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。 ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。 バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
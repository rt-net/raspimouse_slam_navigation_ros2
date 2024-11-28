# raspimouse_slam_navigation
Raspberry Pi MouseでSLAMが行える`raspimouse_slam`パッケージとナビゲーションが行える`raspimouse_navigation`パッケージです。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_setting_goalpose.gif width=500 />

その他のRaspberry Pi MouseのROS 2サンプル集は[rt-net/raspimouse_ros2_examples](https://github.com/rt-net/raspimouse_ros2_examples)で紹介しています。

現在、以下のROS 2のディストリビューションに対応しております。
 - Humble ([humble](https://github.com/rt-net/raspimouse_slam_navigation_ros2/tree/humble))
 - Jazzy ([jazzy](https://github.com/rt-net/raspimouse_slam_navigation_ros2/tree/jazzy)) (This branch)

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
Raspberry Pi MouseとRemote PCを用意しましょう。
Raspberry Pi Mouseにはデバイスドライバをインストールしている必要があります。こちらの[RT Software Turtorials](https://rt-net.github.io/tutorials/raspimouse/driver/install.html)をご参照ください。

以下のリストは、必要なソフトや対応しているセンサなどの一覧を示します。

 - [Raspberry Pi Mouse V3](https://rt-net.jp/products/raspberrypimousev3/)
   - Raspberry Pi - Raspberry Pi 4 Model B
     - Raspberry Pi 3 では動作確認していません
   - Linux OS - Ubuntu 24.04
   - Device Driver - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
   - ROS - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
   - オプションパーツ
     - [Raspberry Pi4用コネクタ](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3776)
     - [マルチLiDARマウント](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)

 - Remote PC
   - Linux OS - Ubuntu 24.04
   - ROS - Jazzy Jalisco

また、本パッケージは以下の機材に対応しています。
 - ゲームパッド: [raspimouse_ros2_examplesのREADME参照](https://github.com/rt-net/raspimouse_ros2_examples#requirements-1)
 - レーザ即域センサ
   - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)
   - [LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3676)
   - [URG-04LX-UG01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1296&products_id=2816)

<a name="Installation"></a>
## Installation
### Raspberry Pi Mouse
以下のコマンドをRaspberry Pi Mouse側で実行してインストールを行います。
```sh
cd ~/ros2_ws/src
# Clone the ROS packages
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_ros2_examples
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_description
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_slam_navigation_ros2
# Install dependencies
rosdep install -y -i --from-paths . --ignore-src

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
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_description
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_slam_navigation_ros2
# Install dependencies
rosdep install -y -i --from-paths . --ignore-src

# make and install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

<a name="QuickStart"></a>
## QuickStart
無事インストールが完了したら、以下の一連のコマンドを実行しましょう。SLAMで地図生成を行い、その地図を利用してRaspberry Pi Mouseを自律移動させます。それぞれの詳しい動かし方などについては[SLAM](#slam)、[ナビゲーション](#navigation)を参照してください。
ここでは例として、ゲームパッドのLogicool F710とレーザ測域センサのRPLIDAR A1を使用しています。

### SLAM
```sh
## ロボット側で以下のコマンドを実行
## ゲームパッドの操作方法については、 https://github.com/rt-net/raspimouse_ros2_examples#joystick_control を参照してください
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

<a name="SLAM"></a>
## raspimouse_slam
[slam_toolboxパッケージ](https://github.com/SteveMacenski/slam_toolbox)
を使用してSLAM（自己位置推定と地図生成）を行うパッケージです。
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2_with_raspimouse_model.png width=500 />

ここでは、ゲームパッドとしてLogicool Wireless Gamepad F710を使用しています。

実際にSLAMを行っている様子は以下のGIF画像にて確認できます。
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.gif width=500 />

### Usage
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

Remote PC上で次のコマンドを実行して、SLAMを開始します。 RVizが立ち上がり、Raspberry Pi Mouseを動かすと地図が構築されていく様子が見られます。

Raspberry Pi MouseとRemote PCが通信するため、同じネットワーク上で同じ`ROS_DOMAIN_ID`を指定する必要があります。詳しい設定方法についてはこちらの[RT Software Tutorials](https://rt-net.github.io/tutorials/raspimouse/ros/samples.html#raspberry-pipcros)のROS 2タブを開いてご参照ください。
```sh
ros2 launch raspimouse_slam pc_slam.launch.py
```

構築した地図をファイルへ保存するために、Remote PC 上で次のコマンドを実行します。
```sh
ros2 run nav2_map_server map_saver_cli -f ~/MAP_NAME
```

コマンドを実行すると`MAP_NAME.pgm`と`MAP_NAME.yaml`の2つのファイルが生成されます。

<a name="Navigation"></a>
## raspimouse_navigation
[Navigation2パッケージ](https://github.com/ros-planning/navigation2)
を使用してRaspberry Pi Mouseを自律移動させるパッケージです。
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_with_raspimouse_model.png width=500 />

また、Raspberry Pi MouseとRemote PCが同じネットワーク上で同じ`ROS_DOMAIN_ID`を指定している必要があります。

### Usage
まずはRaspberry Pi Mouse上で、次のコマンドを実行します。Raspberry Pi MouseのモータとLiDARを起動するためのノードを起動しています。
```sh
# RPLIDAR A1の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=rplidar
# LDS-01の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=lds
# URG-04LX-UG01の場合
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=urg lidar_port:=/dev/ttyACM0
```

Remote PC上で、次のコマンドを実行します。自己位置推定と経路生成用のノードを起動し、RVizを立ち上げます。
引数のmapパラメータには、SLAMで生成した地図（.yamlファイル）を指定してください。
```sh
ros2 launch raspimouse_navigation pc_navigation.launch.py map:=$HOME/MAP_NAME.yaml
```

無事RVizが起動したら、まずは初期位置・姿勢を合わせます。RVizの画面上部の*2D Pose Estimate*をクリックしましょう。
地図上でRaspberry Pi Mouseが存在すべき尤もらしい位置をクリックし、**そのままホールド**します。
ホールドしながらカーソルを動かし、表示されている矢印の向きをRaspberry Pi Mouseの尤もらしい向きに合わせてからボタンを離します。
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_setting_initialpose.gif width=500 />

初期位置・姿勢の指示が完了したら、次は目標位置・姿勢を指示します。RVizの画面上部の*Navigation2 Goal*をクリックしましょう。地図上で、初期位置・姿勢を合わせた時と同様に、地図上をクリックして目標位置を、ホールドしたままカーソルを動かして目標姿勢を指示しましょう。すると、Raspberry Pi Mouseが自律移動を開始します。
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_setting_goalpose.gif width=500 />

### Stopping the robot
下記画像のようなナビゲーション用のパネルがRViz左下に表示されます。
Cancelボタンを押すと自律移動が中断されます。
<img src=https://rt-net.github.io/images/raspberry-pi-mouse/navigation_ros2_rviz_panel.png width=300 />

Raspberry Pi Mouseを停止させる別の方法として、モータへの電源供給を止める方法もあります。
安全に気をつけながらRaspberry Pi Mouseに搭載されたスイッチを操作してモータ用電源をOFFにしましょう。

また、次のコマンドを実行すると、ソフトウェア側からモータ電源をON / OFFできます。
```sh
# モータ電源をOFFにする
ros2 service call /motor_power std_srvs/srv/SetBool data:\ false
# モータ電源をONにする
ros2 service call /motor_power std_srvs/srv/SetBool data:\ true
```


<a name="License"></a>
# License

(C) 2022 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。 ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。 バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。

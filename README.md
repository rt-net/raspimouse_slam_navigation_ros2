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
Raspberry Pi Mouse V3とRemote PCを用意しましょう。
Raspberry Pi Mouse V3にはデバイスドライバをインストールしている必要があります。こちらの[RT Software Turtorials](https://rt-net.github.io/tutorials/raspimouse/driver/install.html)をご参照ください。
また、ロボットとRemote PCは、同じネットワーク上で同じ`ROS_DOMAIN_ID`を指定する必要があります。詳しい設定方法についてはこちらの[RT Software Tutorials](https://rt-net.github.io/tutorials/raspimouse/ros/samples.html#raspberry-pipcros)のROS 2タブを開いてご参照ください。
以下のリストは、必要なソフトや対応しているセンサなどの一覧を示します。  

 - [Raspberry Pi Mouse V3]()
   - Raspberry Pi - Raspberry Pi 4 Model B
   - Linux OS - Ubuntu 20.04
   - Device Driver - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
   - ROS - [Foxy Fiztroy](https://docs.ros.org/en/foxy/Installation.html)
   - Raspberry Pi Mouse ROS Pakcages
     - [rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2) (本パッケージ)
     - [rt-net/raspimouse_ros2_examples](https://github.com/rt-net/raspimouse_ros2_examples)
     - [rt-net/raspimouse_description](https://github.com/rt-net/raspimouse_description/tree/foxy-devel)
   - オプションパーツ
     - [Raspberry Pi4用コネクタ](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3776)
     - [マルチLiDARマウント](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)

 - Remote PC
   - Linux OS - Ubuntu 20.04
   - ROS - [Foxy Fiztroy](https://docs.ros.org/en/foxy/Installation.html)
   - Raspberry Pi Mouse ROS Packages
     - [rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2) (本パッケージ)
     - [rt-net/raspimouse_ros2_examples](https://github.com/rt-net/raspimouse_ros2_examples)

また、本パッケージは以下の機材に対応しています。  
 - ゲームパッド
   - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
   - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)
 - レーザ測域センサ
   - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)

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
# Install dependencies
rosdep install -r -y -i --from-paths . --ignore-src

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
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_descriptions
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_slam_navigation_ros2
# Install dependencies
rosdep install -r -y -i --from-paths . --ignore-src

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
## 新しい端末を開いて次のコマンドを実行しましょう 
cd ~/ros2_ws/src/raspimouse_slam_navigation_ros2/raspimouse_slam
mkdir maps && cd maps
## $MAP_NAMEを地図ファイルの名前に置き換えましょう
ros2 run nav2_map_server map_saver_cli -f $MAP_NAME

# Navigation
## 作ったマップを登録するため、もう一度ビルドします
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
LiDARをつかってSLAM（自己位置推定と地図生成）を行うパッケージです。  

ここでは、レーザ測域センサとして[RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)、ゲームパッドとして[Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)を使用しています。

### Usage
Raspberry Pi Mouse上で、次のコマンドを実行します。LiDARを起動し、ゲームパッドでRaspberry Pi Mouseを制御することができます。  
ゲームパッドの操作方法については、[raspimouse_ros2_examplesの"joystick_control"](https://github.com/rt-net/raspimouse_ros2_examples#joystick_control)を参照してください。  
```sh
ros2 launch raspimouse_slam robot_bringup.launch.py lidar:=rplidar lidar_port:=/dev/ttyUSB0 joyconfig:=f710
```

次のコマンドを実行して、SLAMを開始します。 RVizが立ち上がり、Raspberry Pi Mouseを動かすと地図が構築されていく様子が見られます。  

Remote PC上で起動することを推奨します。この時、Remote PCとRaspberry Pi Mouseが同じネットワーク上で同じROS_DOMAIN_IDを指定している必要があります。  
```sh
ros2 launch raspimouse_slam raspimouse_slam.launch.py lidar:=rplidar
```

構築した地図を保存するために、次のROSノードを起動します。Remote PC上で起動することを推奨します。  
```sh
cd ~/ros2_ws/src/raspimouse_slam_navigation2/raspimouse_slam
mkdir maps && cd maps 
ros2 run nav2_map_server map_saver_cli -f $MAP_NAME
```

`.pgm`と`.yaml`の2つのファイルが生成されます。  
```sh
~/ros2_ws/raspimouse_slam_navigation_ros2/raspimouse_slam/maps$ ls
$MAP_NAME.pgm $MAP_NAME.yaml
```

### Video


<a name="Navigation"></a>
## raspimouse_navigation
SLAMで地図を生成した後、その地図を使って自己位置推定を行い、地図上の任意の座標まで自律移動を行います。  

ここでは、レーザ測域センサとして[RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)を使用しています。  
また、Raspberry Pi MouseとRemote PCが同じネットワーク上で同じROS_DOMAIN_IDを指定している必要があります。  

### Usage
まずはRaspberry Pi Mouse上で、次のコマンドを実行します。Raspberry Pi MouseのモータとLiDARを起動するためのノードを起動しています。  
```sh
ros2 launch raspimouse_navigation robot_navigation.launch.py lidar:=rplidar
```

Remote PC上で、次のコマンドを実行します。自己位置推定と経路生成用のノードを起動し、RVizを立ち上げます。  
引数のmap_fileパラメータには、SLAMで生成した地図（.yamlファイル）を指定してください。  
```sh
ros2 launch raspimouse_navigation map_file:=$MAP_FILE.yaml
```

無事RVizが起動したら、まずは初期位置・姿勢を合わせます。RVizの画面上部の緑色の矢印2D Pose Estimateをクリックしましょう。地図上で、ロボット実機が最もらしい位置までマウスを持ってきてクリックしそのままホールドします。大きな矢印が出ている状態で、マウスを動かすと向きを指示することが可能なので、最もらしい向きに合わせてから、ボタンを離しましょう。  
picture...  

初期位置・姿勢の指示が完了したら、次は目標位置・姿勢を指示します。RVizの画面上部の紫色の矢印2D Nav Goalをクリックしましょう。地図上で、初期位置・姿勢を合わせた時と同様に、地図上をクリックして目標位置を、ホールドしたままマウスを動かして目標姿勢を指示しましょう。すると、ロボットが自律移動を開始します。  
picture...  

### Stopping the robot
TODO トピックでゴールのキャンセルをする方法を書く

ロボットが予期しない挙動をした場合は、安全に気をつけながらRaspberry Pi Mouse V3のモータ用電源をOFFにしましょう。 モータ用電源はRaspberry Pi Mouse V3に搭載されたスイッチでON / OFFできます。 あるいは、次のコマンドを実行すると、ソフトウェアスイッチでモータ電源をOFFにできます。与えた目標位置・姿勢への移動を停止したい場合は、新しいターミナルで次のコマンドを実行しましょう。RViz上には目標位置・姿勢が残りますが、ロボットは停止します。新たに、2D Nav Goalを設置すると、そちらに目標位置・姿勢が置き換わります。  
```sh
ros2 service call /motor_power std_srvs//Bool data:\ false\
```  

### Video

## Notes
本パッケージで使用されているRPLIDAR A1制御用のパッケージはコミュニティがメンテナンスしているパッケージになります。  
公式のパッケージ（[Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)）がリリースされたら置き換える予定です。  


<a name="License"></a>
# License

(C) 2022 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。 ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。 バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
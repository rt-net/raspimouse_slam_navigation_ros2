# raspimouse_navigation_examples

[raspimouse_navigation_examples](./../raspimouse_navigation_examples)パッケージを使用したナビゲーションのサンプルプログラムパッケージです。

## Table of Contents

- [raspimouse\_navigation\_examples](#raspimouse_navigation_examples)
  - [Table of Contents](#table-of-contents)
  - [How To Use Examples](#how-to-use-examples)
    - [Waypoint Navigation](#waypoint-navigation)
      - [Usage](#usage)
  - [Parameters](#parameters)

## How To Use Examples


### Waypoint Navigation

[raspimouse_navigation_examples](./raspimouse_navigation_examples)パッケージを使用してWaypointによるナビゲーションをします。

**本サンプルのWaypointは、シミュレーション上のサンプル地図を想定した４点が設定されています。実機動作時は、ナビゲーション対象の環境に合わせたwaypointを指定してください。**

#### Usage

[raspimouse_navigation_examples/waypoint.py](./raspimouse_navigation_examples/raspimouse_navigation_examples/waypoint.py)コード内のや各Waypointに任意の座標と車体角度を指定します。

- 初期位置

  ```python
    # Initial pose
    initial_pose = generate_pose(navigator=nav, x=0.0, y=0.0, deg=0.0)
    nav.setInitialPose(initial_pose)
```

- Waypoints

  ```python
    # Set goal_1
    goal_poses = []
    goal_pose1 = generate_pose(navigator=nav, x=2.2, y=1.8, deg=0.0)
    goal_poses.append(goal_pose1)
    # Set goal_2
    goal_pose2 = generate_pose(navigator=nav, x=2.8, y=1.0, deg=45.0)
    goal_poses.append(goal_pose2)
    # Set goal_3
    goal_pose3 = generate_pose(navigator=nav, x=2.0, y=0.5, deg=180.0)
    goal_poses.append(goal_pose3)
    # Set goal_4
    goal_pose4 = generate_pose(navigator=nav, x=1.0, y=0.6, deg=180.0)
    goal_poses.append(goal_pose4)
  ```

[ナビゲーション](#navigation)を実行した状態で、Remote PC上の新規ターミナルで以下のコマンドを実行します。

```bash
ros2 launch raspimouse_navigation_examples example.launch.py example:=waypoint
```

コマンドを実行すると、指定したwaypointを通る経路でナビゲーションが開始されます。

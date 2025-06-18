# raspimouse_navigation_examples

Nav2の[Simple Commander API](https://docs.nav2.org/commander_api/index.html)を使用したサンプルパッケージです。

## Table of Contents

- [raspimouse\_navigation\_examples](#raspimouse_navigation_examples)
  - [Table of Contents](#table-of-contents)
  - [How To Use Examples](#how-to-use-examples)
    - [Waypoint Navigation](#waypoint-navigation)
      - [Usage](#usage)
  - [Parameters](#parameters)

## How To Use Examples

### Waypoint Navigation

Waypointによるナビゲーションを行います。

[<img src=https://rt-net.github.io/images/raspberry-pi-mouse/raspimouse_waypoint_navigation_short.gif width=500 />](https://www.youtube.com/watch?v=k2rlcGPZG1M)

**Waypoint Navigationサンプルで設定している4つのWaypointは、サンプルとして[Lake House](https://app.gazebosim.org/OpenRobotics/fuel/models/Lake%20House)を想定して設定されています。実機動作時は、ナビゲーション対象の環境に合わせたwaypointを指定してください。**

#### Usage

[raspimouse_navigation_examples/waypoint.py](./raspimouse_navigation_examples/waypoint.py)内で、各Waypointの位置姿勢を設定しています。`generate_pose()`の引数を変更し、動作環境に合わせた位置姿勢を設定してください。

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
  goal_pose1 = generate_pose(navigator=nav, x=0.0, y=0.5, deg=90.0)
  goal_poses.append(goal_pose1)
  # Set goal_2
  goal_pose2 = generate_pose(navigator=nav, x=1.0, y=0.5, deg=0.0)
  goal_poses.append(goal_pose2)
  # Set goal_3
  goal_pose3 = generate_pose(navigator=nav, x=1.0, y=-0.5, deg=-90.0)
  goal_poses.append(goal_pose3)
  # Set goal_4
  goal_pose4 = generate_pose(navigator=nav, x=0.0, y=-0.5, deg=180.0)
  goal_poses.append(goal_pose4)
  ```

[ナビゲーション](../raspimouse_navigation/README.md#navigation)を実行した状態で、Remote PC上の新規ターミナルで以下のコマンドを実行します。

```bash
ros2 launch raspimouse_navigation_examples example.launch.py example:=waypoint
```

コマンドを実行すると、指定したWaypointを順に走行します。

90秒以内に最後のWaypointに到達できない場合は初期位置に戻り、180秒経過しても完了しない場合はその場でナビゲーションタスクを終了します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/raspimouse_waypoint_navigation_rviz.png width=500 />

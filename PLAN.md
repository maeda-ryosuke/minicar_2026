# minicar_gazebo 進捗と計画
環境: Ubuntu 22.04 / AMD Radeon 780M (Mesa, GPU描画) / ROS2 Humble + Gazebo Harmonic / Docker。

---

## これまでの進捗（完了）

- **段階1**: ROS2 Humble + Gazebo Harmonic の Docker 環境を AMD Radeon で構築。
  `/dev/dri` パススルー + Mesa で GPU 描画に成功。公式 Ackermann 車 `vehicle_blue` を表示。
- **段階2**: ROS2 → `ros_gz_bridge` → Gazebo の連携を確認。`ros2 topic pub` の `cmd_vel` で
  車両が走ること、odometry が ROS2 に返ることを確認（ROS↔gz 双方向 OK）。
- **段階3**: 車両に 2D Lidar + IMU を搭載、障害物ワールドを作成。Lidar が障害物を検知
  （640本中100本が有限値）、IMU も publish。ブリッジ後 `/scan`(10Hz)・`/imu`(100Hz) を ROS2 で確認。
  - 成果物: [worlds/ackermann_sensors.sdf](worlds/ackermann_sensors.sdf)（デフォルト world）

### 途中で判明した重要な事実（ハマりどころ）

- IMU の gz 型は **`gz.msgs.IMU`（大文字）**。`gz.msgs.Imu` だとブリッジが失敗する。
- この Humble+Harmonic 版 `ros_gz_bridge` は **YAML 設定も同梱 launch も持たない**。
  `parameter_bridge` は CLI 引数のみ。→ 自前 launch で引数を渡す方式にする。
- Lidar/IMU を動かすには world に `gz-sim-sensors-system`(ogre2) と `gz-sim-imu-system`
  プラグインが必須（同梱 ackermann world には無い）。
- `gz sim` がコンテナのメインプロセスなので、GUI を閉じるとコンテナごと終了する（正常挙動）。

## その先の段階（未着手）

1. **TF ツリーの整備**: SLAM には `map→odom→base_link→laser` の TF が必要。
   gz の `/model/vehicle_blue/tf`(`gz.msgs.Pose_V`→`tf2_msgs/msg/TFMessage`) のブリッジや
   `static_transform_publisher` を用意する。`/scan` の frame は `vehicle_blue/chassis/gpu_lidar`。
2. **SLAM**: `slam_toolbox` に `/scan` + TF を入力してマッピング。
3. **MPPI**: Nav2 の `nav2_mppi_controller`（alitekes1 で設定済みの構成を参考に）。
4. **alitekes1 合流検討**: SLAM/MPPI/IMU/Nav2 が統合済みの
   `alitekes1/ackermann-vehicle-gzsim-ros2`（Harmonic）を土台にするか。この段階で ament
   パッケージ化を導入。

## 参考（別プロジェクト）

- 内部の詳細計画: `~/.claude/plans/ros2humble-...-eager-harbor.md`
- 過去のトラブル記録: `~/Docker/f1tenth_gym_ros_2026-07-15.md`

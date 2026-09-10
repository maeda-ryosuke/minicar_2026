# minicar_gazebo

ROS 2 Humble + Gazebo Harmonic (gz-sim 8) を Docker で動かす最小構成。
AMD Radeon (Mesa) の GPU パススルーで Gazebo GUI を表示する。

- **確認済み環境**: Ubuntu 22.04 / AMD Radeon 780M (gfx1103, Mesa 23.2, OpenGL 4.6) / XWayland (`DISPLAY=:0`)
- **段階1のゴール**: world を起動し、公式 Ackermann 車両 `vehicle_blue` を表示・操作する

---

## 前提（ログインごとに1回）

コンテナから X11 に接続できるよう許可する。

```bash
xhost +local:
```

---

## 1. ビルド

```bash
cd ~/Docker/minicar_gazebo
docker compose build
```

---

## 2. 起動（world + GUI）

```bash
docker compose up
```

- 別ウィンドウで Gazebo が開き、青い Ackermann 車 (`vehicle_blue`) が地面に出る。
- ログはこの端末に流れる。**停止は `Ctrl-C`**（またはウィンドウを閉じる）。
- `-r` 付きで起動しているので最初から物理シミュレーションが走る状態。

別の world を試す場合（同梱 world は下記「参考」参照）:

```bash
docker compose run --rm gz gz sim -v4 -r diff_drive.sdf
```

---

## 3. 車両を動かす / 状態を見る（別端末で）

`gz topic` は **動いているコンテナの中で** 実行する。上の `docker compose up` を動かしたまま、別の端末で:

### トピック一覧

```bash
docker compose exec gz gz topic -l
```

### 前進 + 旋回させる（cmd_vel を送信）

```bash
docker compose exec gz gz topic -t /model/vehicle_blue/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.1}"
```

- `linear.x` = 前後速度 [m/s]、`angular.z` = 旋回 [rad/s]。
- 停止させるには 0 を送る:

```bash
docker compose exec gz gz topic -t /model/vehicle_blue/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"
```

### オドメトリを受信（echo）

```bash
docker compose exec gz gz topic -e -t /model/vehicle_blue/odometry
```

### あるトピックの型・接続先を調べる

```bash
docker compose exec gz gz topic -i -t /model/vehicle_blue/cmd_vel
```

---

## 4. ROS2 から動かす（ros_gz_bridge）

ROS2 側から送った `cmd_vel` を **ros_gz_bridge** 経由で Gazebo に届け、車両を動かす。
ここでは ROS2 ノードは書かず、`ros2 topic pub` の手打ちだけで経路を確認する。

> **要点**: ブリッジも `ros2 topic pub` も **Gazebo と同じコンテナ内**で実行する
> （`docker compose exec` で相乗り）。同一コンテナなら gz transport(共有メモリ)と
> ROS2 DDS が自動で噛み合い、ネットワーク設定が不要。以下は `docker compose up` で
> シミュレータが起動している前提。

### 4-1. ブリッジを起動（別端末で）

`cmd_vel` を ROS→gz、`odometry` を gz→ROS で橋渡しする。この端末は起動したままにする。

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 run ros_gz_bridge parameter_bridge \
  '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'"
```

方向記号（ROS型と gz型の間の文字）:

| 記号 | 向き | 用途 |
| --- | --- | --- |
| `@` | 双方向 | どちらからでも |
| `[` | gz → ROS | センサ等を ROS 側で受け取る（例: odometry） |
| `]` | ROS → gz | 指令を gz 側へ送る（例: cmd_vel） |

### 4-2. ROS2 からコマンドを送る（さらに別端末で）

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 topic pub -r 10 /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.2}}'"
```

- `-r 10` = 10Hz で送り続ける（走行を持続）。単発なら `--once`。
- 停止は `Ctrl-C`。念のため 0 を送ってもよい:
  `... '{linear: {x: 0.0}, angular: {z: 0.0}}'`
- world 側で速度は ±1 m/s に制限されている（`linear.x` は 1.0 で頭打ち）。

### 4-3. Gazebo が受け取ったことを確認する

**方法A: ブリッジ経由の odometry を ROS2 側で見る**（4-1 で odometry も張ってある前提）

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 topic echo --once /model/vehicle_blue/odometry"
```
コマンド送信中に `position` の `x`/`y` が増えていけば、ROS2→gz→（動作）→gz→ROS2 が
一周通っている証拠。

**方法B: gz 側で直接 cmd_vel を覗く**（ブリッジより手前で届いているか切り分け）

```bash
docker compose exec gz gz topic -e -t /model/vehicle_blue/cmd_vel
```
ROS2 から publish した Twist がここに流れていれば、ブリッジは正常に ROS→gz している。

**方法C: 画面で見る**
Gazebo ウィンドウで `vehicle_blue` が前進しつつ旋回すれば成功。

### 切り分けのヒント

動かないときは、まずブリッジを介さず段階3の
`docker compose exec gz gz topic -t /model/vehicle_blue/cmd_vel -m gz.msgs.Twist -p "..."`
が効くか確認する。効けば gz 側は正常で、原因はブリッジか ROS2 側に絞れる。

---

## 5. センサ（Lidar / IMU）

**前提**: `docker compose up` の既定 world は `worlds/minicar_course_traced.sdf`
（実コースの壁 + ミニカー `tt02` に 2D Lidar と IMU を搭載）。センサやブリッジの学習用に
`worlds/ackermann_sensors.sdf`（`vehicle_blue` + 障害物）もあり、CLI で切替可能:
`docker compose run --rm gz gz sim -v4 -r ackermann_sensors.sdf`。
以下のトピック名/frame 名は既定の course world（`tt02`）基準。`vehicle_blue` の world
を使う場合はモデル名を読み替えること。

### ミニカーモデル `tt02`（Tamiya TT-02 相当）

- 実体: [worlds/models/tt02/model.sdf](worlds/models/tt02/model.sdf)。course world に `<include>`。
- 実寸: ホイールベース **257mm**・トレッド **166mm**・タイヤ径 **63mm**・全長 ~0.38m・質量 ~1.5kg。
  140cm レーンを走れる小型車（`vehicle_blue` は全長2mで大きすぎたため差替）。
- 構成: Ackermann 4輪（前輪 steering_link 経由）+ `gpu_lidar`（前方・地上~0.10m）+ `imu`。
  操舵指令は `/model/tt02/cmd_vel`（Twist）。odometry は `/model/tt02/odometry`。

world に加えてあるもの:
- world 直下の system プラグイン: `gz-sim-sensors-system`(ogre2) と `gz-sim-imu-system`
  → これが無いと Lidar/IMU は動かない。
- `chassis` リンクの `gpu_lidar`（2D, 270°/640点, 0.1–30m, 10Hz, topic `/scan`）と
  `imu`（100Hz, topic `/imu`）。
- 静的障害物（前方の箱・左の箱・右の円柱）。

### 5-1. gz 側でセンサが出ているか確認

```bash
docker compose exec gz gz topic -l | grep -iE "scan|imu"
# /scan  /scan/points  /imu
```

Lidar の値を直接見る（障害物方向は有限値、何もない方向は inf）:

```bash
docker compose exec gz gz topic -e -t /scan -n 1 | grep -m5 "ranges:"
```

### 5-2. ROS2 へブリッジ（別端末で）

**IMU の gz 型は `gz.msgs.IMU`（大文字）**。`gz.msgs.Imu` と書くと
`Failed to create a bridge` になるので注意。

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 run ros_gz_bridge parameter_bridge \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'"
```

**通常は `docker compose up` に含まれる（手動起動は不要）**

デフォルトの `command`（[run_sim.sh](run_sim.sh)）が Gazebo と一緒に
[launch/gz_bridge.launch.py](launch/gz_bridge.launch.py) を起動する。よって
`docker compose up` するだけで、以下が ROS2 に出る:

- `/scan`（LaserScan, 10Hz） `/imu`（100Hz） `/odom`（`/model/.../odometry` を remap）
- `/tf`（`tt02/odom → tt02/chassis`。gz の tf をブリッジ）
- `/joint_states`（左右前輪の実操舵角。操舵測定用）
- 静的変換 `tt02/chassis → tt02/chassis/gpu_lidar`（lidar 取付 chassis 相対 x=0.10, z=0.045）
- **`/cmd_vel`（Twist, ROS→gz）** — ROS2 側から車両を動かす指令経路

`/cmd_vel` だけ方向記号が `]`（ROS→gz）で、他と逆向き。remap は ROS 側の名前にしか
効かないので、gz 側は AckermannSteering 既定の `/model/tt02/cmd_vel` のまま。

手動で個別に張りたい場合（切り分け用）は launch を直接叩いてもよい:

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 launch /launch/gz_bridge.launch.py"
```

> メモ: このバージョンの `ros_gz_bridge` は YAML 設定/同梱 launch を持たないため、
> launch 内で `parameter_bridge` に CLI 引数を渡す方式。IMU の gz 型は大文字 `gz.msgs.IMU`。

### 5-2b. rviz2 で /scan を可視化

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && rviz2"
```

rviz2 の設定:
- **Fixed Frame = `tt02/odom`**（`odom` ではない。frame 名は `tt02/` 付き）
- Add → By topic → `/scan` の LaserScan を追加 → コースの壁の位置に点が並ぶ。

点群が出ないときは TF が原因。`ros2 run tf2_ros tf2_echo tt02/odom
tt02/chassis/gpu_lidar` で変換が引けるか、`/tf` の Publisher count が 1 以上かを確認。

### 5-3. ROS2 側で確認

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /scan"   # ~10Hz
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /imu"    # ~100Hz
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 topic echo --once /scan | grep -E 'frame_id|angle_min|range_max'"
# frame_id: tt02/chassis/gpu_lidar
```

### 5-4. 障害物検知の確証

車両を障害物へ寄せる（section 4 の cmd_vel を使用）と、その方向の `ranges` が短くなる。
GUI では `visualize=true` の Lidar 光線が障害物に当たって短くなるのが見える。

### トラブル切り分け

- **Lidar/IMU のトピックが出ない**: world に Sensors/Imu プラグインがあるか、
  `docker compose logs | grep -i sensor` で `SensorsPrivate::RenderThread started` が出ているか確認。
- **ogre2 が AMD で不安定**: `worlds/ackermann_sensors.sdf` の `<render_engine>ogre2` を
  `ogre` に下げる。GUI 描画とセンサ描画は別経路なので、GUI は出るのにセンサだけ落ちる場合はここ。

---

## 5-5. FTG 自律走行と可視化

### 走らせる

```bash
docker compose up                      # 端末1: Gazebo + ブリッジ

# 端末2: FTG + MPPI + 安全フィルタ（本命）
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 launch /launch/mppi.launch.py"

# 比較用: FTG + Pure Pursuit 版
#   ros2 launch /launch/ftg.launch.py
```

操舵の切り分け測定では、通常の `mppi.launch.py` の代わりに次を使う。
joint/IMU/scan/真値を事前確認してから制御を開始し、最初の膠着で自動停止する。

```bash
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 launch /launch/mppi_measurement.launch.py"
```

結果はホストの `logs/<日時>/trace.csv` と `summary.txt` に残る。CSVには
MPPI舵角、safety通過後の舵角、左右実舵角、IMU実角速度、真値、FTG目標、
各入力の鮮度を10Hzで記録する。測定中のloggerは制御トピックをpublishしない。

真値での走行距離: **Pure Pursuit 10.63m / MPPI 44.97m**（どちらもコース内）。
MPPI は 1.8 秒先までロールアウトするので「今わずかに外へ膨らんで後で
曲がりきる」という複数ステップの計画ができる。Pure Pursuit は今この瞬間の
目標点しか見ないため、ヘアピンで気づいたときには曲がりきれない。
壁際では終端距離に加え、各予測位置からFTG目標点を見た方位と車体姿勢の
誤差も評価する。距離場から作った危険度と近時刻優先の重みを掛けることで、
減速だけを残して旋回を後半へ先送りする候補を不利にする。

車へ出す舵角は最適化した制御列の先頭 `u_0` そのもの（論文 Williams et al. 2017
の Alg.1 `SendToActuators(u_0)`）。操舵の一次遅れ（gz の AckermannSteering は
比例制御なので τ=1/steer_p_gain=0.067秒）はロールアウト側のモデルに残してあり、
指令には掛けない。遅れ適用後の値を publish すると gz 側の遅れと二重になって、
ロールアウトの前提と実車の挙動がずれる。

MPPI のコア(`nodes/mppi_core.py`)は ROS 非依存なので、シミュレータ抜きで
テストできる:

```bash
docker run --rm -v $PWD/nodes:/nodes:ro -w /nodes minicar_gazebo:latest \
  python3 test_mppi_core.py
```

指令チェーンは以下。`safety_node` は制御方式に依存しないので、`pursuit_node`
を MPPI に差し替えるときもそのまま使う。

```
ftg_node --/ftg/target_point--> mppi_node --/cmd_vel_raw-->
safety_node --/cmd_vel--> bridge --> gz
```

`safety_node` の責務はウォッチドッグ（制御ノードが落ちても止まる）、
スタック検知と後退脱出、加速度制限。

### 見る（rviz2）

```bash
xhost +local:
# 端末3
docker compose exec gz bash -c "source /opt/ros/humble/setup.bash && \
  ros2 launch /launch/viz.launch.py"
```

| 表示 | トピック | 意味 |
|---|---|---|
| 白い点群 | `/scan` | センサが見ている全部（270°） |
| 水色の点群 | `/ftg/scan_filtered` | 前処理後（前方180°・5mクリップ・バブル塗り） |
| 緑の球＋線＋数値 | `/ftg/markers` | FTG の目標点と**目標角度[deg]・最接近距離[m]** |
| 緑の線 | `/viz/path_true` | 真の軌跡（シミュレータ内の実位置） |
| 赤の線 | `/viz/path_odom` | odom の軌跡 |
| 灰の細線 | `/mppi/markers` | MPPI のサンプル軌道（既定 30 本） |
| 橙の太線 | 同上 | MPPI の採用軌道 |
| 薄い色地 | `/mppi/esdf` | 距離場。壁に近いほど濃い |

白と水色を重ねると、FTG が何を捨てたか（後方 33% のビーム、5m 超、
安全バブルで塗った範囲）が見える。

### 緑と赤が離れたら odom が嘘をついている

**`/odom` は AckermannSteering が車輪回転と操舵角から積分した推測航法で、
実測ではない。** 壁に押し付けられて車輪が空転していても「指令どおり走行中」
を返し続ける（実測: odom が 0.49rad/s のとき IMU は 0.000）。

実際にこれで測定を誤り、車が止まっているのに「55m 走行」と判断したことが
ある（真値は 6.45m）。別の走行では **11m 走った時点で odom の位置が真値から
7.5m ずれた**。走行距離の差は 1.4m しかないのに位置が 7.5m ずれるのは、
空転中に方位が狂い、以降ずっと誤った向きに積分し続けるため。

評価には必ず `/ground_truth/odom`（`gz-sim-odometry-publisher-system` が
真の姿勢から作る）を使うこと。ただし**実機には存在しない情報なので、
制御ノードから購読してはいけない**。

## 5-6. キーボードで手動走行（地図作成用）

SLAM の地図作成は低速で 2〜3 周する必要がある。FTG/MPPI の自律走行は速度も
ラインも地図作成向きではない（速すぎる・壁に突っ込む）ので、手で走らせる。

```bash
docker compose exec gz bash /teleop.sh
```

起動すると旋回半径と舵角が表示される。キーは
`i` 前進 / `,` 後退 / `k` 停止 / `u`,`o` 前進+旋回 / `m`,`.` 後退+旋回。

### 加減速は必ず `q` / `z` を使う

`q`/`z` は speed と turn を**同時に**スケールするので `angular.z / linear.x`
の比が保たれる。gz の AckermannSteering は `delta = atan(omega*L/v)` で舵角を
作るため、**舵角はこの比だけで決まる**。比が変わらなければ旋回半径も変わらない。

`w`/`x`（linear のみ）と `e`/`c`（angular のみ）は比を壊す。`x` で 0.5→0.3 m/s
に落とすと比が 1.5→2.5 になり、内輪が上限 0.5 rad に張り付いて**それ以上
曲がらなくなる**（切り増しても無反応）。

### 既定値を変えるときは「内輪」で判定する

飽和するのは bicycle model の平均舵角ではなく**内輪**（`atan(L/(R-kingpin/2))`）。
平均が上限内でも内輪が先に張り付く。steering joint の実測:

| 比 (turn/speed) | 旋回半径 | 内輪 | 外輪 | 平均(理論) | |
|---|---|---|---|---|---|
| 1.00 | 1.00m | 0.270 | 0.236 | 0.252 | |
| **1.50** | **0.67m** | **0.407** | **0.336** | **0.368** | ← 既定 |
| 1.80 | 0.56m | 0.487 | 0.390 | 0.433 | 限界直前 |
| 2.00 | 0.50m | 0.504 | 0.401 | 0.475 | **飽和** |
| 3.00 | 0.33m | 0.504 | 0.401 | 0.657 | 飽和（切り増し無効） |

飽和境界は比 1.85。`teleop_twist_keyboard` のパッケージ既定 `turn=1.0`
（`speed=0.5` に対し比 2.0）は飽和するので、`teleop.sh` では 0.75 にしてある。

上書きは環境変数で:

```bash
docker compose exec -e SPEED=0.3 -e TURN=0.45 gz bash /teleop.sh
```

バナーが内輪舵角と飽和判定を出すので、変更したらそこを見ること。

### 自律走行の launch と同時に上げない

`safety_node` は**タイマー駆動で 20Hz 無条件に `/cmd_vel` を publish する**
（`/cmd_vel_raw` が来ていなければゼロ Twist）。teleop も `/cmd_vel` に出すので、
`ftg.launch.py` / `mppi.launch.py` を上げたまま teleop すると publisher が
2 つになって殴り合い、車がガタつく。`teleop.sh` は起動時に検出して警告を出す。

なお AckermannSteering は最後の指令をラッチするので、止めないと走り続ける。
`teleop_twist_keyboard` は Ctrl-C でゼロ Twist を送ってから終了するが、
`kill -9` した場合は次で止める:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
```

### 地図作成の通し手順

```bash
# 端末1
docker compose up

# 端末2 — SLAM
docker compose exec gz bash -c \
  "source /opt/ros/humble/setup.bash && ros2 launch /launch/slam_mapping.launch.py"

# 端末3 — 手動走行。低速で 2〜3 周（ループ閉じ込みのため最低 2 周）
docker compose exec gz bash /teleop.sh

# 端末4 — 保存
docker compose exec gz bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 run nav2_map_server map_saver_cli -f /maps/minicar_course"
```

rviz2 で見るなら `config/slam.rviz`（`/map`、`/scan_filtered` と生 `/scan` の
重ね、ポーズグラフ、TF）。

## 6. 停止・後片付け

```bash
# 起動端末で Ctrl-C した後、コンテナを削除
docker compose down
```

強制的に消す場合:

```bash
docker rm -f minicar_gazebo
```

---

## 参考

### GPU パススルーの確認（描画がおかしいとき）

コンテナ内の OpenGL レンダラがホストと同じ `GFX1103_R1` なら GPU 有効。
`llvmpipe` と出たらソフトウェア描画にフォールバックしている。

```bash
docker compose run --rm gz glxinfo -B | grep -iE "OpenGL renderer|OpenGL version"
```

ソフトウェア描画で切り分けたい場合は [docker-compose.yaml](docker-compose.yaml) の
`LIBGL_ALWAYS_SOFTWARE=0` を `1` にする。

### 同梱 world の一覧を見る

```bash
docker compose run --rm gz ls /usr/share/gz/gz-sim8/worlds/
```

代表例: `ackermann_steering.sdf`（Ackermann車）, `diff_drive.sdf`（差動2輪）,
`empty.sdf`（空）, `shapes.sdf`, `sensors.sdf`, `gpu_lidar_sensor.sdf`。

### gz コマンド早見

| やりたいこと | コマンド |
| --- | --- |
| トピック一覧 | `gz topic -l` |
| 送信 (publish) | `gz topic -t <topic> -m <msg型> -p "<内容>"` |
| 受信 (echo) | `gz topic -e -t <topic>` |
| トピック情報 | `gz topic -i -t <topic>` |
| サービス一覧 | `gz service -l` |
| gz-sim バージョン | `gz sim --version` |

いずれも `docker compose exec gz <コマンド>` で実行する。

---

## AMD 向けに変えてある点（NVIDIA構成との違い）

参考にした NVIDIA 前提の構成に対し、この compose は以下を AMD 向けにしている:

- `runtime: nvidia` や `NVIDIA_*` 環境変数は**使わない**
- `/dev/dri` をデバイスとして渡す（Radeon の render ノード）
- ホストの `render`(GID 110) / `video`(GID 44) グループを `group_add` でコンテナに付与
  → GID は環境依存。自分の値は `getent group render video` で確認できる

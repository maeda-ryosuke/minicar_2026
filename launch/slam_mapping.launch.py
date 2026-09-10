"""slam_toolbox で地図を作る（mapping モード）。

Gazebo とブリッジは別（docker compose up が run_sim.sh 経由で上げる）。
ここは SLAM 側だけ。GUI は viz.launch.py と分ける既存方針に合わせ、
rviz2 はここに含めない。

    docker compose exec gz bash -c \
      "source /opt/ros/humble/setup.bash && ros2 launch /launch/slam_mapping.launch.py"

前提（満たしていないと無言で動かない）:
  - /clock がブリッジされ、全ノードが use_sim_time:=true であること
  - odom_tf_node が TF odom -> base_link を出していること
    （slam_toolbox はオドメトリをトピックではなく TF で受け取る）
  - base_link -> スキャンの frame_id への静的 TF があること
  上 3 つはすべて gz_bridge.launch.py が面倒を見ている。

走らせ方:
  1. docker compose up            # Gazebo + bridge + odom_tf + 静的TF
  2. ros2 launch /launch/slam_mapping.launch.py
  3. 低速で 2〜3 周する（ループ閉じ込みのため最低 2 周）
     低速走行は「退化の対策」ではなくスキャン間オーバラップの確保のため。
     センシング半径 2m に対し 10Hz・3m/s では 30cm/スキャン動いてしまう。
  4. 保存:
     ros2 run nav2_map_server map_saver_cli -f /maps/minicar_course
     ros2 service call /slam_toolbox/serialize_map \
       slam_toolbox/srv/SerializePoseGraph "{filename: '/maps/minicar_course'}"
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 実機の壁高 0.10m という物理制約(前方180deg / 2m)をシムにも被せる。
    # ここを飛ばすとシムだけ 270deg / 30m の甘い条件で評価してしまい、
    # 実機に持っていった瞬間に破綻する。
    scan_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join("/launch", "scan_filter.launch.py"))
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=["/config/slam_toolbox_mapping.yaml"],
    )

    return LaunchDescription([scan_filter, slam])

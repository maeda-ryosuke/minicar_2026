"""可視化 — 軌跡パブリッシャ + rviz2。

走行そのものには不要なので ftg.launch.py とは分けてある。手元で目視確認
したいときだけ上げる。GUI が出るので X11 の共有が必要。

    xhost +local:
    docker compose exec gz bash -c \
      "source /opt/ros/humble/setup.bash && ros2 launch /launch/viz.launch.py"

rviz2 に出るもの:
  白い点群   /scan              センサが見ている全部(270deg)
  水色の点群 /ftg/scan_filtered 前処理後(前方180deg・5mクリップ・バブル塗り)
  緑の球と線 /ftg/markers       FTG の目標点。目標角度と最接近距離を数値表示
  緑の線     /viz/path_true     真の軌跡
  赤の線     /viz/path_odom     odom の軌跡。緑から離れた分が odom の嘘
  黄の線     /lap/markers       スタート/フィニッシュ線
  灰/橙の線  /mppi/markers      MPPI のサンプル軌道 / 採用軌道
  薄い色地   /mppi/esdf         MPPI の距離場

周回数・周回タイムは lap_node が端末に出す。終了時(Ctrl-C)に走行結果の
サマリと軌跡 CSV(/tmp/lap_trajectory.csv)を書き出す。
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # use_sim_time は全ノードで揃えること（理由は ftg.launch.py 参照）。
    paths = ExecuteProcess(
        cmd=["python3", "/nodes/viz_node.py",
             "--ros-args", "--params-file", "/config/viz_params.yaml",
             "-p", "use_sim_time:=true"],
        output="screen",
    )
    laps = ExecuteProcess(
        cmd=["python3", "/nodes/lap_node.py",
             "--ros-args", "--params-file", "/config/lap_params.yaml",
             "-p", "use_sim_time:=true"],
        output="screen",
    )
    rviz = ExecuteProcess(
        cmd=["rviz2", "-d", "/config/ftg.rviz",
             "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )
    return LaunchDescription([paths, laps, rviz])

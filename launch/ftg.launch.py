"""FTG 自律走行（ftg_node + pursuit_node）をまとめて起動する。

Gazebo とブリッジは別（docker compose up が run_sim.sh 経由で上げる）。
ここは制御側だけ。MPPI に差し替えるときは pursuit_node を入れ替える。

    docker compose exec gz bash -c \
      "source /opt/ros/humble/setup.bash && ros2 launch /launch/ftg.launch.py"

colcon パッケージを作っていないので Node ではなく ExecuteProcess で
単独スクリプトを起動する。--ros-args 以降はそのまま rclpy に渡る。
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def _node(script: str, params: str):
    # use_sim_time は全ノードで揃えること。gz ブリッジが流すメッセージは
    # シミュレーション時刻(0 起点)で刻まれているので、一部だけ wall clock
    # のままにすると時刻が混在して TF の lookup が壊れる。
    return ExecuteProcess(
        cmd=["python3", script, "--ros-args", "--params-file", params,
             "-p", "use_sim_time:=true"],
        output="screen",
    )


def generate_launch_description():
    # 指令チェーン:
    #   ftg_node --/ftg/target_point--> pursuit_node --/cmd_vel_raw-->
    #   safety_node --/cmd_vel--> bridge --> gz
    #
    # safety_node は制御方式に依存しないので、pursuit を MPPI に差し替える
    # ときもここだけ残す。publisher を1つに保つため、安全機構は並列に
    # publish させず必ずチェーンの途中に挟むこと。
    return LaunchDescription([
        _node("/nodes/ftg_node.py", "/config/ftg_params.yaml"),
        _node("/nodes/pursuit_node.py", "/config/pursuit_params.yaml"),
        _node("/nodes/safety_node.py", "/config/safety_params.yaml"),
    ])

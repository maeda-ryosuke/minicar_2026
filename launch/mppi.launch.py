"""FTG + MPPI 自律走行。

    ftg_node --/ftg/target_point--> mppi_node --/cmd_vel_raw--> safety_node --/cmd_vel--> gz

ftg_node が「どちらへ行きたいか」を1点で示し、mppi_node が 1.8 秒先まで
ロールアウトしてそこへ向かう軌道を選ぶ。壁を避けるだけでなく「今わずかに
外へ膨らんで後で曲がりきる」という複数ステップの計画ができるのが
Pure Pursuit 版との違い。

Pure Pursuit 版(launch/ftg.launch.py)は比較用に残してある。
真値でのベースラインは 10.63m 走って壁に膠着。

    docker compose exec gz bash -c \
      "source /opt/ros/humble/setup.bash && ros2 launch /launch/mppi.launch.py"
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def _node(script: str, params: str):
    # use_sim_time は全ノードで揃えること（理由は ftg.launch.py 参照）。
    return ExecuteProcess(
        cmd=["python3", script, "--ros-args", "--params-file", params,
             "-p", "use_sim_time:=true"],
        output="screen",
    )


def generate_launch_description():
    return LaunchDescription([
        _node("/nodes/ftg_node.py", "/config/ftg_params.yaml"),
        _node("/nodes/mppi_node.py", "/config/mppi_params.yaml"),
        _node("/nodes/safety_node.py", "/config/safety_params.yaml"),
    ])

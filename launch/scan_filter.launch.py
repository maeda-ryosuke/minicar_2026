"""SLAM 用スキャン前処理（前方180deg / 2m ゲート）だけを起動する。

mapping と localization の両方が必要とするので、独立した launch にして
双方から include する。単体で上げてトピックを覗きたいときにも使える:

    docker compose exec gz bash -c \
      "source /opt/ros/humble/setup.bash && ros2 launch /launch/scan_filter.launch.py"
    ros2 topic echo /scan_filtered --once
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", "/nodes/scan_filter_node.py",
                 "--ros-args",
                 "-p", "use_sim_time:=true",
                 "-p", "fov_deg:=360.0",
                 "-p", "range_max:=20.0"],
            output="screen",
        ),
    ])

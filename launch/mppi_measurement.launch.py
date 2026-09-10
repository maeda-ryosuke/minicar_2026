"""MPPI 操舵測定。

logger を先に起動し、joint/IMU/scan/ground truth の事前確認時間を確保してから
既存の mppi.launch.py を起動する。logger が最初の膠着を検出して終了すると
launch 全体も停止し、safety_node の finally がゼロ Twist を送る。

    ros2 launch /launch/mppi_measurement.launch.py
"""

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    logger = ExecuteProcess(
        cmd=[
            "python3", "/nodes/steer_debug.py",
            "--ros-args",
            "--params-file", "/config/steer_measurement_params.yaml",
            "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    controls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("/launch/mppi.launch.py")
    )

    # logger の事前確認は 2.5 秒で失敗終了する。3 秒後に制御を始めることで、
    # センサ配線が壊れた状態では車両を動かさない。
    delayed_controls = TimerAction(period=3.0, actions=[controls])

    stop_all_when_logger_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=logger,
            on_exit=[EmitEvent(event=Shutdown(reason="操舵測定loggerが終了"))],
        )
    )

    return LaunchDescription([
        # logger が起動直後に失敗した場合も制御開始を防げるよう、終了監視を
        # logger / 遅延制御より先に登録する。
        stop_all_when_logger_exits,
        logger,
        delayed_controls,
    ])

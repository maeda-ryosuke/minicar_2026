#!/bin/bash
# docker compose up で「Gazebo + ROS2 ブリッジ」を一括起動するスクリプト。
# ENTRYPOINT(ros_entrypoint.sh)が先に ROS を source 済みなので、ここでは不要。
#
# bridge + static_tf をバックグラウンドで、gz sim をフォアグラウンドで動かす。
# gz sim をメイン(exec)にすることで、GUI を閉じる / compose down でコンテナが正しく終了する。

set -e

# ブリッジ(scan/imu/odom/joint state)と静的変換。gz topic が未生成でも
# lazy に待つので先行でよい。
ros2 launch /launch/gz_bridge.launch.py &

exec gz sim -v4 -r /worlds/minicar_course_traced.sdf

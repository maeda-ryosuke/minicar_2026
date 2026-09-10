#!/bin/bash
# キーボードで tt02 を手動走行させる（地図作成用）。
#
#   docker compose exec gz bash /teleop.sh
#
# docker compose exec は既定で TTY を割り当てるのでキー入力が届く。
# ENTRYPOINT を通らない起動経路なので ROS は自分で source する。
#
# === なぜ launch ファイルにしないか ===
# teleop_twist_keyboard は stdin を termios で raw モードにして読む。
# ros2 launch の ExecuteProcess 配下では stdin が TTY にならずキーを
# 受け取れないため、必ずこのスクリプトから直接起動すること。
#
# === なぜ turn を既定の 1.0 から 0.75 に下げているか ===
# gz の AckermannSteering は Twist から delta = atan(omega*L/v) で舵角を
# 作る。つまり舵角は angular.z / linear.x の「比」だけで決まり、
# 比 = 曲率 = 1/旋回半径 になる。
#
# ただし飽和判定に使うべきは bicycle model の平均舵角ではなく「内輪」。
# Ackermann では内輪が delta_in = atan(L/(R - kingpin/2)) と平均より
# 大きく切れるので、平均が上限内でも内輪が先に張り付く。実測:
#
#   比    R[m]   内輪    外輪   平均(理論)
#   1.00  1.00   0.270  0.236   0.252
#   1.50  0.67   0.407  0.336   0.368     <- 既定。余裕あり
#   1.80  0.56   0.487  0.390   0.433     <- 限界直前
#   2.00  0.50   0.504  0.401   0.475     <- 飽和(パッケージ既定 turn=1.0)
#   3.00  0.33   0.504  0.401   0.657     <- 切り増しても一切変わらない
#
# 飽和境界は比 1.85 (= L/tan(0.5) + kingpin/2 から R>=0.540m)。
# パッケージ既定の speed=0.5/turn=1.0 は比 2.0 で内輪が飽和するため、
# turn を 0.75 (比 1.5) に落として指令どおりの旋回半径が出るようにする。
# コースの通路幅 0.82m に対し R=0.67m は十分小回りが利く
# (FTG/MPPI は r_min=1.0m でも周回できている)。
#
# 速度 0.5m/s は LiDAR 10Hz に対し 5cm/スキャンで、slam_toolbox が要求
# するスキャン間オーバラップも満たす。

set -e
source /opt/ros/humble/setup.bash

SPEED="${SPEED:-0.5}"
TURN="${TURN:-0.75}"

# 実際に出る舵角と旋回半径を計算して見せる。値を変えたときに
# 内輪がシムの steering_limit 0.5rad を超えていないかをここで確認できる。
python3 - "$SPEED" "$TURN" << 'PYEOF'
import math, sys
v, w = float(sys.argv[1]), float(sys.argv[2])
L, KINGPIN, LIMIT = 0.257, 0.14, 0.5   # model.sdf の AckermannSteering と一致
kappa = w / v                          # 曲率 = angular.z / linear.x
R = 1.0 / kappa
delta = math.atan(w * L / v)                    # bicycle model の平均
delta_in = math.atan(L / (R - KINGPIN / 2))     # 内輪。こちらが先に飽和する
ok = "OK" if delta_in < LIMIT else "*** 飽和 (turn を下げること) ***"
print(f"""
================= tt02 keyboard teleop =================
  speed={v} m/s   turn={w} rad/s
  -> 曲率 {kappa:.2f} /m = 旋回半径 {R:.2f} m
  -> 舵角 平均 {delta:.3f} rad / 内輪 {delta_in:.3f} rad
     シム上限 {LIMIT} rad に対し {ok}

  走行   i 前進    , 後退    k 停止
         u / o  前進+左右旋回
         m / .  後退+左右旋回

  加減速 q / z  speed と turn を同時に 1.1 / 0.9 倍   <<< これを使う
         ---------------------------------------------------------
         w / x  linear のみ    e / c  angular のみ
         ^ 比が崩れて旋回半径が変わる。減速すると内輪が飽和する。
           例: x で 0.5->0.3 m/s に落とすと turn は据え置きなので
               比が 1.5 -> 2.5 になり、内輪が 0.5 rad に張り付いて
               それ以上曲がらなくなる(切り増しても無反応になる)。
           q/z なら両方が同率で動くので比が保たれ、旋回半径は不変。

  Ctrl-C で終了（ゼロ Twist を送ってから抜けるので車は止まる）
========================================================
""")
PYEOF

# safety_node もタイマー駆動で 20Hz 無条件に /cmd_vel を publish するため、
# 自律走行 launch が上がっていると publisher が 2 つになって殴り合う。
if ros2 node list 2>/dev/null | grep -q safety_node; then
    echo "!!! safety_node が動いています。ftg.launch.py / mppi.launch.py を"
    echo "!!! 止めてください。/cmd_vel の publisher が 2 つになり車がガタつきます。"
    echo
fi

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:="$SPEED" -p turn:="$TURN"

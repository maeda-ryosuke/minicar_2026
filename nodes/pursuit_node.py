#!/usr/bin/env python3
"""Pure Pursuit — ステップ 2-5。FTG の目標点を追って /cmd_vel_raw を出す。

出力は /cmd_vel ではなく /cmd_vel_raw。最終段の safety_node が素通し・
ウォッチドッグ・スタック脱出・加速度制限を担当し、そこから /cmd_vel が出る。
このノードは「目標点を追う」ことだけに責任を持つ。


FTG(ftg_node) が /ftg/target_point に出す「後輪車軸から見た目標点」を
円弧で追う。ゲイン調整の要る P 制御ではなく幾何解を使う:

    目標点までの距離 l_d と方位 alpha に対し、そこを通る円弧の曲率は
        kappa = 2*sin(alpha) / l_d
    自転車モデルの舵角は
        delta = atan(kappa * L)
    gz の AckermannSteering は Twist(v, omega) から delta = atan(omega*L/v)
    を内部で作るので、omega = v * kappa を渡せば辻褄が合う。

MPPI に差し替えるときは、このノードを止めて MPPI ノードを上げるだけでよい。

単独スクリプトとして動く:
    python3 /nodes/pursuit_node.py --ros-args --params-file /config/pursuit_params.yaml
"""

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PursuitNode(Node):
    def __init__(self):
        super().__init__("pursuit_node")

        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("target_timeout", 0.3)
        self.declare_parameter("v_straight", 1.2)
        self.declare_parameter("v_corner", 0.6)
        self.declare_parameter("vehicle_params_file", "/config/vehicle_params.yaml")
        self.declare_parameter("target_topic", "/ftg/target_point")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        with open(self.get_parameter("vehicle_params_file").value) as f:
            vp = yaml.safe_load(f)
        self.L = float(vp["vehicle"]["wheelbase"])
        self.r_min = float(vp["limits"]["r_min"])
        self.delta_max = float(vp["limits"]["delta_max"])
        self.a_max = float(vp["limits"]["a_max"])
        self.v_limit = float(vp["limits"]["v_max"])
        self.kappa_max = 1.0 / self.r_min

        self.rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / self.rate
        self.timeout = float(self.get_parameter("target_timeout").value)

        self._target = None      # (x, y)
        self._target_time = None
        self._v_prev = 0.0


        self.sub = self.create_subscription(
            PointStamped, self.get_parameter("target_topic").value, self.on_target, 10
        )
        self.pub = self.create_publisher(
            Twist, self.get_parameter("cmd_topic").value, 10
        )
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"pursuit_node started. L={self.L} r_min={self.r_min} "
            f"delta_max={self.delta_max:.4f} kappa_max={self.kappa_max:.3f} "
            f"rate={self.rate}Hz"
        )

    def on_target(self, msg: PointStamped) -> None:
        self._target = (msg.point.x, msg.point.y)
        self._target_time = self.get_clock().now()

    def stop(self) -> None:
        """ゼロ指令。AckermannSteering は最後の指令を保持し続けるため、
        止めたいときは明示的に 0 を送らないと走り続ける。"""
        self._v_prev = 0.0
        self.pub.publish(Twist())

    def on_timer(self) -> None:
        # FTG が gap 無し(緊急)と判断すると目標点の publish が止まる。
        # 一定時間来なければ停止に落とす。ノードが落ちた場合も同じ経路で止まる。
        if self._target is None or self._target_time is None:
            self.stop()
            return
        age = (self.get_clock().now() - self._target_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.stop()
            self.get_logger().warn(
                f"target stale ({age:.2f}s > {self.timeout}s). STOP",
                throttle_duration_sec=1.0,
            )
            return

        x, y = self._target
        ld = float(np.hypot(x, y))
        if ld < 1e-3:
            self.stop()
            return

        # --- Pure Pursuit の幾何解 ---
        alpha = float(np.arctan2(y, x))
        kappa = 2.0 * np.sin(alpha) / ld

        # 車の限界を超える曲率は出せない。ここで頭打ちにしておかないと
        # gz 側が勝手に飽和させ、予測と実挙動がずれる。
        kappa = float(np.clip(kappa, -self.kappa_max, self.kappa_max))
        delta = float(np.clip(np.arctan(kappa * self.L),
                              -self.delta_max, self.delta_max))

        # --- 速度: 曲率が大きいほど落とす ---
        v_str = float(self.get_parameter("v_straight").value)
        v_cnr = float(self.get_parameter("v_corner").value)
        ratio = abs(kappa) / self.kappa_max
        v_target = v_str - (v_str - v_cnr) * ratio
        v_target = float(np.clip(v_target, 0.0, self.v_limit))

        # 加速度制限は safety_node(最終段)が持つ。ここでかけると、あちらが
        # 脱出のため指令を上書きしている間に _v_prev が実際とずれ、素通しに
        # 戻った瞬間に速度が飛ぶ。アクチュエータ直前でかけるのが正しい。
        v = v_target

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * kappa   # omega = v/R = v*kappa
        self.pub.publish(cmd)

        self.get_logger().info(
            f"alpha={np.degrees(alpha):+6.2f}deg ld={ld:.2f}m "
            f"kappa={kappa:+.3f} delta={np.degrees(delta):+6.2f}deg "
            f"v={v:.2f} omega={cmd.angular.z:+.3f}",
            throttle_duration_sec=1.0,
        )


def main() -> None:
    rclpy.init()
    node = PursuitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 終了時に必ず止める。これが無いと Ctrl-C 後も最後の指令のまま
        # 走り続ける（AckermannSteering は指令をラッチする）。
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

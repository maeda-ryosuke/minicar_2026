#!/usr/bin/env python3
"""周回判定 — シミュレータの真の姿勢だけを使う評価ノード。

判定に使うのは /ground_truth/odom のみ。これは gz-sim-odometry-publisher-system
がシミュレータ内の真の姿勢から作るもので、スリップも衝突もそのまま出る。
/odom（車輪回転と操舵角からの推測航法）は絶対に使わない — 壁に押し付けられて
空転していても「指令どおり走行中」を返し続け、実際に「真値 6.45m の走行を
55m」と誤報告した実績がある。

コースは周回ではなく蛇行(サーペンタイン)なので、中心周りの角度積算では
判定できない。スポーン地点にスタート/フィニッシュ線を引き、

    1. 線を進行方向に横切ったか（逆走で戻ってきたのは数えない）
    2. 線の近傍を通ったか（無限直線なので遠方での交差を弾く）
    3. 前回通過から min_lap_distance 以上走ったか（線の上でうろついても
       周回にならないようにする）

の3条件で数える。

軌跡は CSV にも落とす。周回できていないときに「どこで詰まったか」を
オフラインで確かめられるようにするため。

    python3 /nodes/lap_node.py --ros-args --params-file /config/lap_params.yaml
"""

import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class LapNode(Node):
    def __init__(self):
        super().__init__("lap_node")

        # world SDF の <include><pose>5.23 1.0 0.05 0 0 0</pose> と揃えること。
        self.declare_parameter("spawn_x", 5.23)
        self.declare_parameter("spawn_y", 1.0)
        self.declare_parameter("spawn_yaw", 0.0)
        # 線の近傍とみなす横方向の幅[m]。スタート線は無限直線なので、
        # コースの反対側で交差しても数えないように絞る。
        self.declare_parameter("gate_half_width", 2.0)
        # 前回通過からこれだけ走らないと次の周回として数えない[m]。
        # 蛇行コースの1周はざっと 40〜60m。スタート線の上で往復しても
        # 周回にならないようにするための下限。
        self.declare_parameter("min_lap_distance", 20.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("csv_path", "/tmp/lap_trajectory.csv")
        self.declare_parameter("report_period", 5.0)

        self.sx = float(self.get_parameter("spawn_x").value)
        self.sy = float(self.get_parameter("spawn_y").value)
        yaw = float(self.get_parameter("spawn_yaw").value)
        # n = 進行方向。線はこれに直交する。s = (p - spawn)·n が線からの符号付き距離。
        self.nx, self.ny = math.cos(yaw), math.sin(yaw)
        # t = 線に沿う方向。ゲート幅の判定に使う。
        self.tx, self.ty = -self.ny, self.nx
        self.gate = float(self.get_parameter("gate_half_width").value)
        self.min_lap = float(self.get_parameter("min_lap_distance").value)
        self.frame = self.get_parameter("world_frame").value

        self._p = None
        self._s_prev = None
        self._dist = 0.0          # 総走行距離
        self._lap_dist = 0.0      # 今の周回に入ってからの距離
        self._armed = False       # min_lap_distance を超えたら通過を数え始める
        self._t0 = None
        self._lap_t0 = None
        self._laps = []           # [(周回番号, 所要時間, 距離)]
        self._rows = []

        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 10)
        self.pub_marker = self.create_publisher(MarkerArray, "/lap/markers", 1)
        self.create_timer(float(self.get_parameter("report_period").value),
                          self.report)
        self.create_timer(1.0, self.publish_line)

        self.get_logger().info(
            f"lap_node started. start/finish=({self.sx}, {self.sy}) "
            f"yaw={math.degrees(yaw):.0f}deg gate=±{self.gate}m "
            f"min_lap={self.min_lap}m"
        )

    # ------------------------------------------------------------------
    def on_gt(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = self._lap_t0 = now

        if self._p is not None:
            d = math.hypot(p.x - self._p[0], p.y - self._p[1])
            self._dist += d
            self._lap_dist += d
        self._p = (p.x, p.y)
        self._rows.append((now - self._t0, p.x, p.y, self._dist))

        dx, dy = p.x - self.sx, p.y - self.sy
        s = dx * self.nx + dy * self.ny        # 線からの符号付き距離(進行方向が正)
        lat = abs(dx * self.tx + dy * self.ty)  # 線に沿う方向のずれ

        if self._lap_dist >= self.min_lap:
            self._armed = True

        if (self._armed and self._s_prev is not None
                and self._s_prev < 0.0 <= s and lat <= self.gate):
            # 進行方向に、線の近傍で、十分走ってから横切った = 1周
            lap_t = now - self._lap_t0
            self._laps.append((len(self._laps) + 1, lap_t, self._lap_dist))
            self.get_logger().error(          # 目立たせたいので error レベル
                f"★ {len(self._laps)} 周目 完了  所要 {lap_t:.1f}s  "
                f"距離 {self._lap_dist:.2f}m  (総 {self._dist:.2f}m)"
            )
            self._lap_dist = 0.0
            self._lap_t0 = now
            self._armed = False

        self._s_prev = s

    # ------------------------------------------------------------------
    def report(self) -> None:
        if self._p is None:
            self.get_logger().warn("ground_truth 未受信")
            return
        el = 0.0 if self._t0 is None else (
            self.get_clock().now().nanoseconds * 1e-9 - self._t0)
        self.get_logger().info(
            f"t={el:5.1f}s 位置=({self._p[0]:+6.2f},{self._p[1]:+6.2f}) "
            f"総走行={self._dist:6.2f}m 周回={len(self._laps)} "
            f"今周={self._lap_dist:5.2f}m {'(判定待機)' if self._armed else ''}"
        )

    def publish_line(self) -> None:
        """スタート/フィニッシュ線を rviz に出す。"""
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id = "lap", 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
        m.points = [
            Point(x=self.sx + self.tx * self.gate, y=self.sy + self.ty * self.gate),
            Point(x=self.sx - self.tx * self.gate, y=self.sy - self.ty * self.gate),
        ]
        ma.markers.append(m)
        self.pub_marker.publish(ma)

    def finish(self) -> None:
        path = self.get_parameter("csv_path").value
        try:
            with open(path, "w") as f:
                f.write("t,x,y,dist\n")
                for r in self._rows:
                    f.write("%.3f,%.4f,%.4f,%.4f\n" % r)
        except Exception as e:
            self.get_logger().warn(f"CSV 書き出しに失敗: {e}")

        print("\n===== 走行結果 (真値) =====")
        print(f"総走行距離: {self._dist:.2f} m")
        print(f"完了周回数: {len(self._laps)}")
        for n, t, d in self._laps:
            print(f"  {n} 周目: {t:.1f}s / {d:.2f}m")
        if not self._laps:
            print(f"  周回未完了。今周の走行 {self._lap_dist:.2f}m "
                  f"(判定に必要な最低距離 {self.min_lap}m)")
        if self._p:
            print(f"最終位置: ({self._p[0]:+.2f}, {self._p[1]:+.2f})")
        print(f"軌跡 CSV: {path} ({len(self._rows)} 点)")


def main() -> None:
    rclpy.init()
    node = LapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

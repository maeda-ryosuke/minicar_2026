#!/usr/bin/env python3
"""MPPI 操舵指令の連鎖を記録する観測専用ノード。

制御には一切関与せず、次の4段を10Hzで同じ行へ記録する。

    ① MPPI が要求した平均舵角       /cmd_vel_raw
    ② safety 通過後の平均舵角       /cmd_vel
    ③ Gazebo の左右前輪実舵角       /joint_states
    ④ 車体が実際に回頭した角速度    /imu

ヘアピン区間を後から特定できるよう /ground_truth/odom と
/ftg/target_point も併記する。/odom は評価に使用しない。

CSV は毎周期逐次 flush する。safety_node と同じ膠着条件が1秒続くと終了し、
summary.txt を確定する。測定launchはこのプロセスの終了を受けて制御一式を止める。
"""

import csv
from datetime import datetime
import math
from pathlib import Path
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan


CSV_COLUMNS = [
    "sim_time_s", "elapsed_s",
    "raw_v_mps", "raw_omega_rps", "mppi_delta_rad", "mppi_delta_deg",
    "saturation_pct",
    "cmd_v_mps", "cmd_omega_rps", "cmd_delta_rad", "cmd_delta_deg",
    "joint_left_rad", "joint_left_deg", "joint_right_rad", "joint_right_deg",
    "imu_omega_z_rps",
    "gt_x_m", "gt_y_m", "gt_yaw_rad", "gt_yaw_deg",
    "ftg_target_x_m", "ftg_target_y_m", "ftg_bearing_rad", "ftg_bearing_deg",
    "scan_delta_m",
    "age_raw_s", "age_cmd_s", "age_joint_s", "age_imu_s",
    "age_gt_s", "age_target_s", "age_scan_s",
    "stuck_state",
]

REQUIRED_SIGNALS = ("raw", "cmd", "joint", "imu", "gt", "target", "scan")
PREFLIGHT_SIGNALS = ("joint", "imu", "gt", "scan")


class SteerDebug(Node):
    def __init__(self):
        super().__init__("steer_debug")

        self.declare_parameter("rate", 10.0)
        self.declare_parameter("output_root", "/logs")
        self.declare_parameter("world_file", "/worlds/minicar_course_traced.sdf")
        self.declare_parameter("model_name", "tt02")
        self.declare_parameter("vehicle_params_file", "/config/vehicle_params.yaml")
        self.declare_parameter("safety_params_file", "/config/safety_params.yaml")
        self.declare_parameter("freshness_limit", 0.25)
        self.declare_parameter("preflight_timeout", 2.5)
        self.declare_parameter("control_start_timeout", 10.0)
        self.declare_parameter("left_joint", "front_left_wheel_steering_joint")
        self.declare_parameter("right_joint", "front_right_wheel_steering_joint")

        with open(self.get_parameter("vehicle_params_file").value) as fp:
            vehicle_params = yaml.safe_load(fp)
        with open(self.get_parameter("safety_params_file").value) as fp:
            safety_params = yaml.safe_load(fp)["safety_node"]["ros__parameters"]

        self.wheelbase = float(vehicle_params["vehicle"]["wheelbase"])
        self.delta_max = float(vehicle_params["limits"]["delta_max"])
        self.stuck_cmd_threshold = float(safety_params["stuck_cmd_threshold"])
        self.stuck_imu_threshold = float(safety_params["stuck_imu_threshold"])
        self.stuck_v_threshold = float(safety_params["stuck_v_threshold"])
        self.stuck_scan_threshold = float(safety_params["stuck_scan_threshold"])
        self.stuck_time = float(safety_params["stuck_time"])

        self.rate = float(self.get_parameter("rate").value)
        self.freshness_limit = float(self.get_parameter("freshness_limit").value)
        self.preflight_timeout = float(self.get_parameter("preflight_timeout").value)
        self.control_start_timeout = float(
            self.get_parameter("control_start_timeout").value)

        self._raw = None
        self._cmd = None
        self._joint_left = None
        self._joint_right = None
        self._imu_omega = None
        self._gt = None
        self._target = None
        self._prev_scan = None
        self._scan_delta = None
        self._seen = {name: None for name in REQUIRED_SIGNALS}

        self._sim_start = None
        self._preflight_ready = False
        self._preflight_ready_at = None
        self._control_ready = False
        self._stuck_since = None
        self._stuck_reason = ""
        self._rows = []
        self._finished = False
        self._stop_reason = "手動停止またはlaunch終了"
        self.exit_code = 0
        self._wall_started = datetime.now().astimezone()

        run_id = self._wall_started.strftime("%Y%m%d_%H%M%S_%f")
        self.run_dir = Path(str(self.get_parameter("output_root").value)) / run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        # コンテナは root で動くが、測定後はホスト利用者がログを整理できる
        # 必要がある。run directory の削除・追記権限だけ明示的に開ける。
        self.run_dir.chmod(0o777)
        self.csv_path = self.run_dir / "trace.csv"
        self.summary_path = self.run_dir / "summary.txt"
        self._csv_fp = self.csv_path.open(
            "w", newline="", buffering=1, encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_fp, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()

        self.create_subscription(Twist, "/cmd_vel_raw", self.on_raw, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(
            Imu, "/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/ground_truth/odom", self.on_ground_truth, 10)
        self.create_subscription(PointStamped, "/ftg/target_point", self.on_target, 10)
        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f"操舵測定logger started: {self.run_dir} "
            f"L={self.wheelbase:.4f}m delta_max={math.degrees(self.delta_max):.2f}deg"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _mark(self, name: str) -> None:
        self._seen[name] = self._now_s()

    def on_raw(self, msg: Twist) -> None:
        self._raw = (float(msg.linear.x), float(msg.angular.z))
        self._mark("raw")

    def on_cmd(self, msg: Twist) -> None:
        self._cmd = (float(msg.linear.x), float(msg.angular.z))
        self._mark("cmd")

    def on_joints(self, msg: JointState) -> None:
        left_name = str(self.get_parameter("left_joint").value)
        right_name = str(self.get_parameter("right_joint").value)
        for name, position in zip(msg.name, msg.position):
            if name == left_name:
                self._joint_left = float(position)
            elif name == right_name:
                self._joint_right = float(position)
        if self._joint_left is not None and self._joint_right is not None:
            self._mark("joint")

    def on_imu(self, msg: Imu) -> None:
        self._imu_omega = float(msg.angular_velocity.z)
        self._mark("imu")

    def on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)
        if self._prev_scan is not None and self._prev_scan.size == ranges.size:
            self._scan_delta = float(np.abs(ranges - self._prev_scan).mean())
        self._prev_scan = ranges
        self._mark("scan")

    @staticmethod
    def _yaw_of(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def on_ground_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._gt = (float(p.x), float(p.y), self._yaw_of(msg))
        self._mark("gt")

    def on_target(self, msg: PointStamped) -> None:
        x, y = float(msg.point.x), float(msg.point.y)
        self._target = (x, y, math.atan2(y, x))
        self._mark("target")

    def _age(self, name: str, now_s: float) -> float:
        stamp = self._seen[name]
        return float("inf") if stamp is None else max(0.0, now_s - stamp)

    def _fresh(self, name: str, now_s: float) -> bool:
        return self._age(name, now_s) <= self.freshness_limit

    def _delta_of(self, command) -> float:
        if command is None:
            return float("nan")
        velocity, omega = command
        if abs(velocity) < 1e-3:
            return float("nan")
        return math.atan(omega * self.wheelbase / velocity)

    @staticmethod
    def _value(value) -> float:
        return float("nan") if value is None else float(value)

    def _preflight(self, now_s: float) -> bool:
        if all(self._fresh(name, now_s) for name in PREFLIGHT_SIGNALS):
            self._preflight_ready = True
            self._preflight_ready_at = now_s
            self.get_logger().info(
                "事前確認OK: /joint_states /imu /ground_truth/odom /scan")
            return True

        if now_s - self._sim_start >= self.preflight_timeout:
            missing = [name for name in PREFLIGHT_SIGNALS if not self._fresh(name, now_s)]
            self._request_stop(
                "事前確認失敗: " + ", ".join(missing), exit_code=2)
        return False

    def _stuck_state(self, now_s: float) -> str:
        # safety_node は入力側の /cmd_vel_raw で膠着判定してから、加速度制限を
        # 掛けた /cmd_vel を出す。同じ判定にするため、ここも raw を使う。
        if self._raw is None:
            self._stuck_since = None
            return "none"

        cmd_v, cmd_omega = self._raw
        turning_but_still = (
            self._fresh("imu", now_s)
            and abs(cmd_omega) > self.stuck_cmd_threshold
            and abs(self._value(self._imu_omega)) < self.stuck_imu_threshold
        )
        driving_but_still = (
            self._fresh("scan", now_s)
            and abs(cmd_v) > self.stuck_v_threshold
            and self._scan_delta is not None
            and self._scan_delta < self.stuck_scan_threshold
        )

        if not (turning_but_still or driving_but_still):
            self._stuck_since = None
            self._stuck_reason = ""
            return "none"

        if self._stuck_since is None:
            self._stuck_since = now_s
            self._stuck_reason = "旋回不能" if turning_but_still else "前進不能"
        held = now_s - self._stuck_since
        if held >= self.stuck_time:
            return f"detected:{self._stuck_reason}"
        return f"candidate:{self._stuck_reason}:{held:.2f}s"

    def _make_row(self, now_s: float, stuck_state: str) -> dict:
        raw_v, raw_omega = self._raw or (float("nan"), float("nan"))
        cmd_v, cmd_omega = self._cmd or (float("nan"), float("nan"))
        raw_delta = self._delta_of(self._raw)
        cmd_delta = self._delta_of(self._cmd)
        saturation = (
            abs(raw_delta) / self.delta_max * 100.0
            if math.isfinite(raw_delta) else float("nan")
        )
        gt_x, gt_y, gt_yaw = self._gt or (float("nan"),) * 3
        target_x, target_y, target_bearing = self._target or (float("nan"),) * 3
        joint_left = self._value(self._joint_left)
        joint_right = self._value(self._joint_right)

        row = {
            "sim_time_s": now_s,
            "elapsed_s": now_s - self._sim_start,
            "raw_v_mps": raw_v,
            "raw_omega_rps": raw_omega,
            "mppi_delta_rad": raw_delta,
            "mppi_delta_deg": math.degrees(raw_delta),
            "saturation_pct": saturation,
            "cmd_v_mps": cmd_v,
            "cmd_omega_rps": cmd_omega,
            "cmd_delta_rad": cmd_delta,
            "cmd_delta_deg": math.degrees(cmd_delta),
            "joint_left_rad": joint_left,
            "joint_left_deg": math.degrees(joint_left),
            "joint_right_rad": joint_right,
            "joint_right_deg": math.degrees(joint_right),
            "imu_omega_z_rps": self._value(self._imu_omega),
            "gt_x_m": gt_x,
            "gt_y_m": gt_y,
            "gt_yaw_rad": gt_yaw,
            "gt_yaw_deg": math.degrees(gt_yaw),
            "ftg_target_x_m": target_x,
            "ftg_target_y_m": target_y,
            "ftg_bearing_rad": target_bearing,
            "ftg_bearing_deg": math.degrees(target_bearing),
            "scan_delta_m": self._value(self._scan_delta),
            "stuck_state": stuck_state,
        }
        for name in REQUIRED_SIGNALS:
            row[f"age_{name}_s"] = self._age(name, now_s)
        return row

    def _print_row(self, row: dict) -> None:
        print(
            f"t={row['elapsed_s']:6.1f}s "
            f"MPPI={row['mppi_delta_deg']:+6.2f}deg "
            f"sat={row['saturation_pct']:5.0f}% "
            f"cmd={row['cmd_delta_deg']:+6.2f}deg "
            f"joint=({row['joint_left_deg']:+6.2f},{row['joint_right_deg']:+6.2f})deg "
            f"omega={row['cmd_omega_rps']:+6.3f}/{row['imu_omega_z_rps']:+6.3f} "
            f"gt=({row['gt_x_m']:+5.2f},{row['gt_y_m']:+5.2f}) "
            f"{row['stuck_state']}",
            flush=True,
        )

    def tick(self) -> None:
        now_s = self._now_s()
        if self._sim_start is None:
            self._sim_start = now_s

        if not self._preflight_ready and not self._preflight(now_s):
            return

        if not self._control_ready:
            self._control_ready = all(
                self._seen[name] is not None for name in ("raw", "cmd", "target"))
            if (not self._control_ready
                    and now_s - self._preflight_ready_at >= self.control_start_timeout):
                missing = [name for name in ("raw", "cmd", "target")
                           if self._seen[name] is None]
                self._request_stop(
                    "制御トピック確認失敗: " + ", ".join(missing), exit_code=2)
                return

        stuck_state = self._stuck_state(now_s)
        row = self._make_row(now_s, stuck_state)
        self._writer.writerow(row)
        self._rows.append(row)
        self._print_row(row)

        if stuck_state.startswith("detected:"):
            reason = stuck_state.split(":", 1)[1]
            self._request_stop(f"膠着検出: {reason}", exit_code=0)

    def _request_stop(self, reason: str, exit_code: int) -> None:
        if self._finished:
            return
        self._stop_reason = reason
        self.exit_code = exit_code
        self.get_logger().error(reason)
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def _finite(rows, key: str) -> np.ndarray:
        values = np.asarray([row[key] for row in rows], dtype=float)
        return values[np.isfinite(values)]

    @classmethod
    def _stat_line(cls, rows, key: str, label: str) -> str:
        values = cls._finite(rows, key)
        if values.size == 0:
            return f"- {label}: データ無し"
        return (
            f"- {label}: max|x|={np.abs(values).max():.4f}, "
            f"mean={values.mean():+.4f}, mean|x|={np.abs(values).mean():.4f}"
        )

    def _coverage_lines(self, drive_rows) -> tuple:
        if not drive_rows:
            return ["- 走行中データ無し"], 0.0
        lines = []
        valid_all = np.ones(len(drive_rows), dtype=bool)
        for name in REQUIRED_SIGNALS:
            ages = np.asarray([row[f"age_{name}_s"] for row in drive_rows])
            valid = np.isfinite(ages) & (ages <= self.freshness_limit)
            valid_all &= valid
            lines.append(f"- {name}: {valid.mean() * 100.0:.1f}%")
        lines.append(f"- 全必須入力が同時に有効: {valid_all.mean() * 100.0:.1f}%")
        return lines, float(valid_all.mean() * 100.0)

    def finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        self._csv_fp.flush()
        self._csv_fp.close()

        wall_ended = datetime.now().astimezone()
        drive_rows = [row for row in self._rows
                      if math.isfinite(row["cmd_v_mps"])
                      and abs(row["cmd_v_mps"]) > 0.05]
        if self._rows:
            end_elapsed = self._rows[-1]["elapsed_s"]
            last_rows = [row for row in drive_rows
                         if row["elapsed_s"] >= end_elapsed - 2.0]
        else:
            last_rows = []

        coverage_lines, combined_coverage = self._coverage_lines(drive_rows)
        valid = (
            self._stop_reason.startswith("膠着検出:")
            and combined_coverage >= 95.0
        )

        delta_diffs = []
        for row in drive_rows:
            raw, cmd = row["mppi_delta_rad"], row["cmd_delta_rad"]
            if math.isfinite(raw) and math.isfinite(cmd):
                delta_diffs.append(abs(raw - cmd))
        diff = np.asarray(delta_diffs, dtype=float)
        diff_line = (
            "- raw→cmd 舵角差: データ無し" if diff.size == 0 else
            f"- raw→cmd 舵角差: max={math.degrees(diff.max()):.3f}deg, "
            f"mean={math.degrees(diff.mean()):.3f}deg"
        )

        saturation = self._finite(drive_rows, "saturation_pct")
        saturation_line = (
            "- MPPI飽和率: データ無し" if saturation.size == 0 else
            f"- MPPI飽和率: max={saturation.max():.1f}%, "
            f"mean={saturation.mean():.1f}%, "
            f"90%以上={(saturation >= 90.0).mean() * 100.0:.1f}%"
        )

        lines = [
            "# MPPI操舵測定 要約",
            "",
            f"- 判定: {'有効' if valid else '無効'}",
            f"- 終了理由: {self._stop_reason}",
            f"- world: {self.get_parameter('world_file').value}",
            f"- model: {self.get_parameter('model_name').value}",
            f"- 実時刻開始: {self._wall_started.isoformat()}",
            f"- 実時刻終了: {wall_ended.isoformat()}",
            f"- サンプル数: {len(self._rows)}（走行中 {len(drive_rows)}）",
            f"- CSV: {self.csv_path}",
            f"- delta_max: {self.delta_max:.5f}rad / {math.degrees(self.delta_max):.3f}deg",
            "",
            "## 全走行統計",
            self._stat_line(drive_rows, "mppi_delta_deg", "MPPI舵角[deg]"),
            self._stat_line(drive_rows, "cmd_delta_deg", "cmd舵角[deg]"),
            saturation_line,
            diff_line,
            self._stat_line(drive_rows, "joint_left_deg", "左実舵角[deg]"),
            self._stat_line(drive_rows, "joint_right_deg", "右実舵角[deg]"),
            self._stat_line(drive_rows, "cmd_omega_rps", "指令角速度[rad/s]"),
            self._stat_line(drive_rows, "imu_omega_z_rps", "IMU実角速度[rad/s]"),
            "",
            "## 膠着直前2秒",
            self._stat_line(last_rows, "mppi_delta_deg", "MPPI舵角[deg]"),
            self._stat_line(last_rows, "cmd_delta_deg", "cmd舵角[deg]"),
            self._stat_line(last_rows, "joint_left_deg", "左実舵角[deg]"),
            self._stat_line(last_rows, "joint_right_deg", "右実舵角[deg]"),
            self._stat_line(last_rows, "cmd_omega_rps", "指令角速度[rad/s]"),
            self._stat_line(last_rows, "imu_omega_z_rps", "IMU実角速度[rad/s]"),
            "",
            f"## データ鮮度（上限 {self.freshness_limit:.2f}s）",
            *coverage_lines,
            "",
            "有効条件: 終了理由が膠着検出、かつ走行中に全必須入力が同時に"
            "有効だった割合が95%以上。",
        ]
        text = "\n".join(lines) + "\n"
        self.summary_path.write_text(text, encoding="utf-8")
        print("\n" + text, flush=True)


def main() -> int:
    rclpy.init()
    node = SteerDebug()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

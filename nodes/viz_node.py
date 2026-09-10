#!/usr/bin/env python3
"""可視化用: 真の軌跡と odom の軌跡を同じ座標系に重ねて出す。

/ground_truth/odom (シミュレータ内の真の姿勢) と /odom (車輪回転と操舵角
からの推測航法) を、どちらも world 座標系の nav_msgs/Path にして publish
する。rviz2 で色分けして重ねると、odom がどこでどれだけ嘘をつき始めたかが
一目で分かる。

なぜ必要か: /odom は壁に押し付けられて車輪が空転していても「指令どおり
走行中」を返し続ける。実際、これを走行距離の評価に使っていたせいで
「55m 走行」と誤報告した(真値は 6.45m、車は止まっていた)。2本を重ねて
おけば、乖離が開いた瞬間に気付ける。

odom の原点はスポーン地点なので、world に載せるにはその分だけ平行移動+
回転させる必要がある。spawn_* パラメータがそれ(world SDF の tt02 の
<include><pose> と一致させること)。

    python3 /nodes/viz_node.py --ros-args -p spawn_x:=5.23 -p spawn_y:=1.0
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class VizNode(Node):
    def __init__(self):
        super().__init__("viz_node")

        # world SDF の <include><pose>5.23 1.0 0.05 0 0 0</pose> と揃える。
        self.declare_parameter("spawn_x", 5.23)
        self.declare_parameter("spawn_y", 1.0)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("max_poses", 4000)

        self.sx = float(self.get_parameter("spawn_x").value)
        self.sy = float(self.get_parameter("spawn_y").value)
        self.syaw = float(self.get_parameter("spawn_yaw").value)
        self.frame = self.get_parameter("world_frame").value
        self.max_poses = int(self.get_parameter("max_poses").value)

        self.path_true = Path()
        self.path_true.header.frame_id = self.frame
        self.path_odom = Path()
        self.path_odom.header.frame_id = self.frame

        self.pub_true = self.create_publisher(Path, "/viz/path_true", 10)
        self.pub_odom = self.create_publisher(Path, "/viz/path_odom", 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_true, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(0.2, self.publish_paths)

        self._t = None
        self._o = None
        self.get_logger().info(
            f"viz_node started. spawn=({self.sx}, {self.sy}, "
            f"{math.degrees(self.syaw):.1f}deg) frame={self.frame}"
        )

    def _append(self, path: Path, x: float, y: float, stamp) -> None:
        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.header.stamp = stamp
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
        if len(path.poses) > self.max_poses:
            del path.poses[0]

    def on_true(self, msg: Odometry) -> None:
        # 真値は最初から world 座標なので変換不要。
        p = msg.pose.pose.position
        self._t = (p.x, p.y)
        self._append(self.path_true, p.x, p.y, msg.header.stamp)

    def on_odom(self, msg: Odometry) -> None:
        # odom はスポーン地点原点なので world へ移す。
        p = msg.pose.pose.position
        c, s = math.cos(self.syaw), math.sin(self.syaw)
        x = self.sx + p.x * c - p.y * s
        y = self.sy + p.x * s + p.y * c
        self._o = (x, y)
        self._append(self.path_odom, x, y, msg.header.stamp)

    def publish_paths(self) -> None:
        now = self.get_clock().now().to_msg()
        self.path_true.header.stamp = now
        self.path_odom.header.stamp = now
        self.pub_true.publish(self.path_true)
        self.pub_odom.publish(self.path_odom)

        # 乖離量をログに出す。数字で見えないと「ずれている」に気付きにくい。
        if self._t and self._o:
            err = math.hypot(self._t[0] - self._o[0], self._t[1] - self._o[1])
            self.get_logger().info(
                f"真値=({self._t[0]:+.2f},{self._t[1]:+.2f}) "
                f"odom=({self._o[0]:+.2f},{self._o[1]:+.2f}) 乖離={err:.2f}m",
                throttle_duration_sec=2.0,
            )


def main() -> None:
    rclpy.init()
    node = VizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

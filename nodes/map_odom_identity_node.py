#!/usr/bin/env python3
"""SLAM が動いていないときだけ map -> odom の恒等変換を出して TF を繋ぐ。

=== なぜ必要か ===

TF ツリーは map -> odom -> base_link -> {laser, imu_link, rear_axle} で、
このうち map -> odom を出すのは slam_toolbox / AMCL である。つまり SLAM を
上げていないと木が 2 つに分断される:

    world -> map                                    <- 静的TF
    odom  -> base_link -> laser / imu_link / ...    <- odom_tf_node と静的TF

この状態で rviz2 の Fixed Frame を world にすると、スキャンもマーカーも
別の木にあるため何も表示されない(実測: viz.launch.py で表示が全部消えた)。

以前は world -> tt02/odom を静的TF で直結していたので SLAM の有無に関係なく
繋がっていた。SLAM を入れた際に map -> odom が SLAM 由来になったことで、
この経路が SLAM 依存になってしまったのが原因。

=== なぜ static_transform_publisher ではなくノードなのか ===

恒等の map -> odom を無条件に流すと、SLAM 起動時に「SLAM」と「こちら」の
2 つが同じ変換を publish して殴り合う。逆に launch 引数で手動切り替えに
すると、切り替え忘れでどちらかの不具合を踏む(実際、既定 false にしていた
せいで viz が無表示になった)。

そこでノードにして、slam_toolbox / AMCL がノードグラフに現れたら自動的に
publish を止める。順序はどちらでもよく、SLAM を落とせば再開する。
利用者が意識する必要が無くなる。

    python3 /nodes/map_odom_identity_node.py --ros-args -p use_sim_time:=true
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MapOdomIdentityNode(Node):
    def __init__(self):
        super().__init__("map_odom_identity")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("rate", 20.0)
        # ここに挙げた文字列を含むノードが居たら自分は黙る。
        self.declare_parameter(
            "yield_to", ["slam_toolbox", "amcl"])

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.yield_to = list(self.get_parameter("yield_to").value)

        self.br = TransformBroadcaster(self)
        rate = float(self.get_parameter("rate").value)
        self.create_timer(1.0 / rate, self.on_timer)
        self.create_timer(0.5, self.on_check)

        self._active = True
        self._owner = None
        self.get_logger().info(
            f"map_odom_identity started: {self.map_frame} -> {self.odom_frame} "
            f"(恒等)。{self.yield_to} が現れたら自動で停止する"
        )

    def on_check(self) -> None:
        """SLAM 系ノードが居るかを見て publish の要否を切り替える。"""
        try:
            names = self.get_node_names()
        except Exception:
            return
        owner = next(
            (n for n in names if any(k in n for k in self.yield_to)), None)

        if owner and self._active:
            self._active = False
            self._owner = owner
            self.get_logger().info(
                f"'{owner}' を検出。{self.map_frame} -> {self.odom_frame} は"
                f"そちらが出すので publish を停止する"
            )
        elif not owner and not self._active:
            self._active = True
            self.get_logger().info(
                f"'{self._owner}' が居なくなった。恒等変換の publish を再開する"
            )
            self._owner = None

    def on_timer(self) -> None:
        if not self._active:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0     # 並進 0 / 回転なしの恒等変換
        self.br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = MapOdomIdentityNode()
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

#!/usr/bin/env python3
"""/odom (nav_msgs/Odometry) を TF odom -> base_link として流す。

なぜ必要か: slam_toolbox も nav2_amcl も、オドメトリを「トピック」ではなく
「TF」で受け取る。どちらも nav_msgs/Odometry を購読せず、odom_frame ->
base_frame の lookupTransform を呼ぶだけである。つまりこのノードが無いと
SLAM は一切動かない。/odom トピックの方は可視化(viz_node)と評価にしか
使われない。

なぜ gz の /tf ブリッジを使わないか: gz の /model/tt02/tf が出すフレーム名は
tt02/odom -> tt02/chassis で固定で、ros_gz_bridge の remappings はトピック名
にしか効かないためフレーム名を書き換えられない。実機側は base_link なので、
そのままでは SLAM のパラメータファイルをシムと実機で共通化できない。

実機への移植: このノードが果たす契約(「/odom 相当の推定値を odom ->
base_link の TF として出す」)は、実機のエンコーダ+IMU オドメトリノードが
果たすべきものとまったく同じ。移植時はこのノードを実機側のものに
差し替えるだけでよい。

タイムスタンプは now() ではなく必ず受信メッセージのものを複製する。gz の
メッセージはシミュレーション時刻(0 起点)で刻まれており、ここで wall clock
を混ぜると TF の時刻整合が崩れて SLAM の lookup が失敗する。

    python3 /nodes/odom_tf_node.py --ros-args -p use_sim_time:=true
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfNode(Node):
    def __init__(self):
        super().__init__("odom_tf_node")

        # フレーム名は外出ししておく。実機で odom/base_link 以外の命名を
        # 使っている場合でもノードを書き換えずに合わせられる。
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.br = TransformBroadcaster(self)
        # gz ブリッジ側の QoS が reliable でも best_effort でも受けられるよう
        # 購読は sensor_data(best_effort) 固定にしておく。逆向きは繋がらない。
        self.create_subscription(
            Odometry, topic, self.on_odom, qos_profile_sensor_data)

        self._n = 0
        self.get_logger().info(
            f"odom_tf_node started: {topic} -> TF "
            f"{self.odom_frame} -> {self.base_frame}"
        )

    def on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        # 受信メッセージの時刻をそのまま使う(now() を使わない)。
        t.header.stamp = msg.header.stamp
        # 入力の frame_id / child_frame_id (gz では tt02/odom, tt02/chassis)は
        # 意図的に無視し、パラメータの名前で上書きする。これがフレーム名を
        # シムと実機で揃えるための肝。
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

        self._n += 1
        if self._n == 1:
            self.get_logger().info("first odom received, publishing TF")


def main() -> None:
    rclpy.init()
    node = OdomTfNode()
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

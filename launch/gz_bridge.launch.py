from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # gz <-> ROS2 のセンサ/指令をまとめてブリッジする専用ノード。
    # Gazebo の起動は別（docker compose up）。ここは橋渡しだけ。
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            # /clock は最優先。ros_gz_bridge が流すメッセージのタイムスタンプは
            # gz のシミュレーション時刻(0 起点)なので、購読側が wall clock の
            # ままだと時刻が 1.7e9 秒ずれる。SLAM は transform_timeout 付きで
            # TF を時刻指定 lookup するため、合わせないと確実に失敗する。
            # 以降すべてのノードで use_sim_time:=true にすること。
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # 方向記号 [ = gz -> ROS。IMU の gz 型は大文字 IMU（Imu だと失敗する）。
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/tt02/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # JointStatePublisher system の出力。操舵指令と実際の左右前輪舵角を
            # 比較するため、測定時は /joint_states が必須。
            "/world/minicar_course_traced/model/tt02/joint_state"
            "@sensor_msgs/msg/JointState[gz.msgs.Model",
            # 方向記号 ] = ROS -> gz。AckermannSteering が購読する速度指令。
            # SDF に <topic> 指定が無いのでプラグイン既定の /model/<name>/cmd_vel。
            "/model/tt02/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # グラウンドトゥルース(真の姿勢)。評価専用で、制御には使わない。
            #
            # /odom は AckermannSteering が車輪回転と操舵角から作る推測航法で
            # あって実測ではない。壁に刺さって車輪が空転していても「順調に
            # 走行中」と報告し続ける（実測: odom が 0.49rad/s を出していると
            # き IMU は 0.000）。走行距離や軌跡の評価に odom を使うと、
            # 止まっている車を「55m 走った」と誤判定する。
            "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            #
            # 注: gz の /model/tt02/tf (gz.msgs.Pose_V) は意図的にブリッジしない。
            # あれが出すフレーム名は tt02/odom -> tt02/chassis で固定であり、
            # bridge の remappings はトピック名にしか効かないのでフレーム名を
            # 書き換えられない。実機(base_link / laser_frame)と名前が揃わず
            # パラメータファイルを共通化できなくなる。代わりに odom_tf_node が
            # /odom から odom -> base_link を出す（実機のオドメトリノードと
            # まったく同じ契約）。
        ],
        remappings=[
            # gz の長い名前を nav 系の慣例に合わせて /odom, /cmd_vel に寄せる。
            # remap は ROS 側の名前にだけ効くので、gz 側は上の長い名前のまま。
            ("/model/tt02/odometry", "/odom"),
            ("/model/tt02/cmd_vel", "/cmd_vel"),
            ("/world/minicar_course_traced/model/tt02/joint_state", "/joint_states"),
        ],
    )

    # /odom (nav_msgs/Odometry) -> TF odom -> base_link。
    # slam_toolbox も AMCL もオドメトリを「トピック」ではなく「TF」で受け取る
    # ので、これが無いと SLAM は一切動かない。
    odom_tf = ExecuteProcess(
        cmd=["python3", "/nodes/odom_tf_node.py",
             "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    def static_tf(name, x, y, z, parent, child):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=name,
            parameters=[{"use_sim_time": True}],
            arguments=["--x", x, "--y", y, "--z", z,
                       "--frame-id", parent, "--child-frame-id", child],
        )

    # world -> map。slam_toolbox / AMCL が map -> odom を出すので、map の親を
    # world にしておくと rviz2 の Fixed Frame を world にしたまま真値・地図・
    # スキャンを同じ座標系で重ねられる。map 原点は最初のスキャン時の車両姿勢
    # ＝スポーン地点なので、値は world SDF の <include><pose> と揃えること。
    #
    # 注: 以前ここは world -> tt02/odom だった。SLAM が map -> odom を出す今、
    # 同じ書き方をすると odom の親が world と map の 2 つになり TF が壊れる。
    world_to_map = static_tf("world_to_map", "5.23", "1.0", "0", "world", "map")

    # gz の LaserScan の frame_id は tt02/chassis/gpu_lidar で固定（bridge では
    # 書き換え不可）。slam_toolbox はスキャンのヘッダからフレーム名を読むので、
    # base_link から繋がってさえいれば葉の名前は何でもよく、実機(laser_frame)
    # とパラメータファイルを共通化できる。値は SDF の chassis 相対 x=0.10 z=0.045。
    lidar_tf = static_tf("base_link_to_lidar", "0.10", "0", "0.045",
                         "base_link", "tt02/chassis/gpu_lidar")

    # bicycle model の原点である後輪車軸。chassis 原点はホイールベース中心
    # (前輪 x=+0.1285 / 後輪 x=-0.1285)なので後輪車軸は x=-0.1285。
    # FTG/MPPI の目標点はこの frame で出す。
    rear_axle_tf = static_tf("base_link_to_rear_axle", "-0.1285", "0", "0",
                             "base_link", "rear_axle")

    # IMU は SDF 上 chassis 原点に置いてあるので恒等変換。
    imu_tf = static_tf("base_link_to_imu", "0", "0", "0",
                       "base_link", "imu_link")

    # map -> odom を出すのは slam_toolbox / AMCL なので、SLAM を上げていない
    # ときは TF ツリーが world->map と odom->base_link の 2 つに分断され、
    # rviz2 に何も表示されなくなる(実測)。このノードが「SLAM が居ないときだけ」
    # 恒等変換で繋ぐ。SLAM が現れたら自動で黙るので、利用者が切り替えを
    # 意識する必要は無い。詳細は nodes/map_odom_identity_node.py の docstring。
    map_odom = ExecuteProcess(
        cmd=["python3", "/nodes/map_odom_identity_node.py",
             "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    return LaunchDescription([
        bridge, odom_tf, world_to_map, lidar_tf, rear_axle_tf, imu_tf,
        map_odom,
    ])

FROM ros:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# OSRF Gazebo apt repo (Harmonic lives here, not in the ROS repo)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget gnupg lsb-release ca-certificates && \
    wget https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic + the Humble<->Harmonic bridge + Mesa test tools.
# ros-humble-ros-gzharmonic pulls gz-harmonic as a dependency, but we name
# gz-harmonic explicitly so the intent is clear.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# SLAM 一式。
#   slam-toolbox     地図作成(mapping)と自己位置推定(localization)
#   nav2-map-server  map_saver_cli で .pgm/.yaml を書き出す
#   rviz2            これまで transitive 依存に頼っていたので明示する
# ros-humble-nav2-amcl は localization フェーズで足す(今は未使用)。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-slam-toolbox \
        ros-humble-nav2-map-server \
        ros-humble-rviz2 && \
    rm -rf /var/lib/apt/lists/*

# 地図作成のための手動走行。FTG/MPPI の自律走行は速度もラインも地図作成
# 向きではない(速すぎる・壁に突っ込む)ので、手で低速に走らせる経路が要る。
# 使い方は /teleop.sh 参照。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-teleop-twist-keyboard && \
    rm -rf /var/lib/apt/lists/*

# docker compose exec は ENTRYPOINT(/ros_entrypoint.sh)を通らないため、
# コンテナのメインプロセスと違って ROS が source されない。対話シェル用に
# .bashrc へ入れておく。
#   docker compose exec gz bash        -> ここが効く
# 非対話(bash -c "...")は .bashrc を読まないので、そちらは
# docker-compose.yaml の BASH_ENV で担当する。
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

ENV DEBIAN_FRONTEND=
CMD ["gz", "sim"]

#!/usr/bin/env python3
"""
ミニカーバトル2026 コース(PDF24p)の Gazebo world 生成 — 画像トレース版。

- 画像の赤線/白線(支柱間の壁)を色判定で自動トレース(traced.json)。
- 較正: 縦横比を独立較正で等方化し、全体縮尺は「一番上の廊下=140cm」で決める。
- v8(generate_course.py)とは別系統。壁は高さ0.2m/厚0.05m。

生成: python3 generate_course_traced.py > minicar_course_traced.sdf
"""
import json
import math
import os

WALL_H, WALL_T, CM = 0.20, 0.05, 0.01
HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = "/tmp/claude-1000/-home-ryosuke-Docker/9cb06b1f-9456-4ae7-b868-049a8116a0b2/scratchpad"

# 支柱検出時の較正(px->cm, 独立x/yで等方化)
X0, X1030 = 1191.6, 3978.6
Y0, Y560 = 1940.7, 193.6
SX = 1030.0 / (X1030 - X0)
SY = 560.0 / (Y0 - Y560)


def load_traced():
    # detect_walls.py がこのスクリプトと同じディレクトリ(worlds/)に書く traced.json を読む
    d = json.load(open(os.path.join(HERE, "traced.json")))
    return d["posts"], d["walls"]


def to_cm(px, py):
    return ((px - X0) * SX, (Y0 - py) * SY)


def h_levels(posts_cm, walls, min_total=250):
    """水平壁を yレベルでまとめ、同レベルの総x延長>=min_total のレベルを上から返す。
    (壁は支柱間=1枚単位なので、レベルごとに合算して長い廊下境界だけ残す)"""
    segs = []
    for (i, j, c) in walls:
        (x1, y1), (x2, y2) = posts_cm[i], posts_cm[j]
        if abs(y2 - y1) < 20 and abs(x2 - x1) > 30:      # 水平セグメント
            segs.append(((y1 + y2) / 2, min(x1, x2), max(x1, x2)))
    segs.sort(key=lambda s: -s[0])
    # yレベルにクラスタ
    clusters = []
    for y, xa, xb in segs:
        placed = False
        for cl in clusters:
            if abs(cl["y"] - y) < 40:
                cl["xs"].append((xa, xb)); cl["y"] = (cl["y"] + y) / 2; placed = True; break
        if not placed:
            clusters.append({"y": y, "xs": [(xa, xb)]})
    levels = []
    for cl in clusters:
        extent = max(b for a, b in cl["xs"]) - min(a for a, b in cl["xs"])
        if extent >= min_total:
            levels.append(cl["y"])
    levels.sort(reverse=True)
    return levels


def top_corridor_cm(posts_cm, walls):
    """最上部の水平廊下 = 最上位の長い水平壁と、その直下の長い水平壁のyギャップ。"""
    levels = h_levels(posts_cm, walls)
    return levels[0] - levels[1]


def wall_link(name, p1, p2, rgba):
    x1, y1 = p1[0] * CM, p1[1] * CM
    x2, y2 = p2[0] * CM, p2[1] * CM
    L = math.hypot(x2 - x1, y2 - y1)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    yaw = math.atan2(y2 - y1, x2 - x1)
    r, g, b, a = rgba
    return f"""    <link name="{name}">
      <pose>{cx:.3f} {cy:.3f} {WALL_H/2:.3f} 0 0 {yaw:.4f}</pose>
      <collision name="c"><geometry><box><size>{L:.3f} {WALL_T} {WALL_H}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{L:.3f} {WALL_T} {WALL_H}</size></box></geometry>
        <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
      </visual>
    </link>"""


def build():
    posts_px, walls = load_traced()
    posts_cm = [to_cm(px, py) for (px, py) in posts_px]
    # 縮尺調整: 一番上の廊下=140cm
    h0 = top_corridor_cm(posts_cm, walls)
    k = 140.0 / h0
    posts_cm = [(x * k, y * k) for (x, y) in posts_cm]
    return posts_cm, walls, h0, k


def main():
    posts_cm, walls, h0, k = build()
    xs = [p[0] for p in posts_cm]
    ys = [p[1] for p in posts_cm]
    W, H = max(xs) - min(xs), max(ys) - min(ys)
    ox, oy = min(xs), min(ys)
    links = []
    for n, (i, j, c) in enumerate(walls):
        rgba = (0.85, 0.1, 0.1, 1) if c == "red" else (0.95, 0.95, 0.95, 1)
        p1 = (posts_cm[i][0] - ox, posts_cm[i][1] - oy)
        p2 = (posts_cm[j][0] - ox, posts_cm[j][1] - oy)
        links.append(wall_link(f"wall_{n}", p1, p2, rgba))
    walls_sdf = "\n".join(links)
    cx, cy = W * CM / 2, H * CM / 2
    import sys
    print(f"<!-- top corridor h0={h0:.1f}cm, scale k={k:.3f}, overall {W:.0f}x{H:.0f}cm -->",
          file=sys.stderr)
    print(f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="minicar_course_traced">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.3 0.2 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="link">
        <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>16 10</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>16 10</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link></model>
    <model name="course_walls"><static>true</static>
{walls_sdf}
    </model>

    <!-- lidar/imu 付きミニカー(別ファイル models/tt02 を include)。TT-02 実寸(全長~0.38m)。
         モデル原点は地面レベルなので spawn z は僅かに浮かせて着地。
         スポーン位置はコース下段の開けた場所(要調整)。GZ_SIM_RESOURCE_PATH=/worlds/models 必要。 -->
    <include>
      <uri>model://tt02</uri>
      <pose>{cx:.2f} 1.0 0.05 0 0 0</pose>
    </include>
  </world>
</sdf>""")


if __name__ == "__main__":
    main()

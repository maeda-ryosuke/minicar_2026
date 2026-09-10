#!/usr/bin/env python3
"""
ミニカーバトル2026 コース(PDF 24ページ)の Gazebo world 生成スクリプト。

- 座標系: 図の左下▲を原点、x=右, y=上, 単位 cm（図の数字をそのまま使う）。
- 図は schematic(正確な縮尺ではない)なので、ピクセルではなく「書かれた寸法」で組む。
- 未記載寸法は「成り行き」。トップビューを PDF に重ねて補正する前提の編集可能テーブル。

出力: minicar_course.sdf （壁=高さ0.2m/厚さ0.05mのbox, P枠=色付き）。
使い方: python3 generate_course.py > minicar_course.sdf
"""

import math

# ---- パラメータ ----
WALL_H = 0.20        # 壁高 [m]
WALL_T = 0.05        # 壁厚 [m]
CM = 0.01            # cm -> m

# ---- 座標テーブル (cm, 原点=左下▲) ----
# 下辺の基準x: 0,250,430,610,750,800,945,1030
# 全体: 1030(横) x 約560(縦)
W, H = 1030.0, 560.0

# 支柱(黄丸)の実測位置 [cm]。PDF24pをレンダリングし黄丸を検出、1030/560で独立較正。
# schematic歪みを縦横別較正で吸収。番号は docs/posts_cm.png と対応。
POSTS = {
    0: (895, 560), 1: (700, 555), 2: (300, 551), 3: (501, 551), 4: (117, 549),
    5: (25, 548), 6: (978, 523), 7: (313, 486), 8: (508, 482), 9: (0, 461),
    10: (1026, 457), 11: (259, 412), 12: (451, 412), 13: (648, 412), 14: (840, 412),
    15: (1, 377), 16: (174, 372), 17: (1019, 357), 18: (1029, 288), 19: (685, 289),
    20: (329, 284), 21: (624, 282), 22: (521, 282), 23: (971, 282), 24: (767, 281),
    25: (1021, 219), 26: (1, 211), 27: (174, 208), 28: (265, 167), 29: (474, 166),
    30: (680, 166), 31: (873, 166), 32: (1030, 120), 33: (17, 118), 34: (84, 58),
    35: (5, 50), 36: (985, 49), 37: (343, 47), 38: (599, 47), 39: (793, 47),
    40: (211, 46), 41: (471, 46),
}

# --- 道路幅の補正(POSTS を使う前に適用) ---
# 横方向の道路=水平壁間の縦ギャップを 140cm に。下辺(y~46)から 140 間隔:
# 内部水平壁を y=186 / 326 / 466 に置き直す(x範囲は支柱のまま)。
for _i in (28, 29, 30, 31):
    POSTS[_i] = (POSTS[_i][0], 186)
for _i in (20, 22, 21, 24, 19):
    POSTS[_i] = (POSTS[_i][0], 326)
for _i in (11, 12, 13, 14):
    POSTS[_i] = (POSTS[_i][0], 466)
# 左の縦方向の道路=左外壁(x=0)〜水色ゾーン右壁 を 170cm に。
for _i in (16, 27):
    POSTS[_i] = (170, POSTS[_i][1])


def P(*idxs):
    return [POSTS[i] for i in idxs]


# 外周ループ(反時計回り, 支柱番号で指定)。
# 下辺 35..36 -> 右辺(面取り)36-32-25-18-17-10-6 -> 上辺 0-1-3-2-4-5
# -> 左辺 5-9-15-26 -> 左下斜め 26-33-34 -> 閉じ
OUTER = P(35, 40, 37, 41, 38, 39, 36,
          32, 25, 18, 17, 10, 6, 0,
          1, 3, 2, 4, 5,
          9, 15, 26, 33, 34, 35)

# 内部壁ポリライン群(支柱ベース)。
INNER = [
    P(15, 16, 27, 26),          # 水色ゾーン枠(右/上/下, 170幅)
    P(16, 11, 12, 13, 14),      # 上段 横壁 y~412
    P(7, 8),                    # ゲート下の短壁
    P(20, 22),                  # "190" 壁(左)
    P(21, 24),                  # オレンジU右側の壁
    P(27, 28),                  # 水色ゾーン下から中段へ
    P(28, 29, 30, 31),          # 中段下 横壁 y~166
    P(23, 31),                  # グレー八角の内側edge
]

# スタートレーン仕切り(縦壁, 垂直): P枠上(y=50)から中段横壁(y=186)へ。
START_DIVIDERS = [
    [(374, 50), (374, 186)],   # スタート1 (P1右)
    [(554, 50), (554, 186)],   # スタート2 (P2右)
    [(734, 50), (734, 186)],   # スタート3 (P3右)
]

# ---- 板長で組み直し ----
# 壁は 180/90/45cm 板。板端+ポスト中心20mm で中心間は 186/95/49cm。
# 支柱ピクセル座標(schematic歪みあり)から、方向を軸スナップし長さを板長に丸める。
BOARDS = [186.0, 95.0, 49.0]        # 中心間 [cm] (180/90/45)
AXIS_SNAP_DEG = 20                   # この角度以内なら水平/垂直に吸着


def _snap_len(L):
    return min(BOARDS, key=lambda b: abs(b - L))


def boardify(pts):
    """支柱ポリライン -> 各セグメントを軸スナップ+板長化して歩き直す。
    先頭点は元位置に固定。方向は元セグメントの向きを軸に吸着。"""
    if len(pts) < 2:
        return list(pts)
    out = [(float(pts[0][0]), float(pts[0][1]))]
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        L = math.hypot(dx, dy)
        ang = math.degrees(math.atan2(dy, dx)) % 360
        snapped = None
        for a in (0, 90, 180, 270):
            d = abs((ang - a + 180) % 360 - 180)
            if d < AXIS_SNAP_DEG:
                snapped = a
                break
        rad = math.radians(snapped) if snapped is not None else math.radians(ang)
        b = _snap_len(L)
        px, py = out[-1]
        out.append((px + b * math.cos(rad), py + b * math.sin(rad)))
    return out


def close_loop(pts):
    """閉ループを板長固定・角度可変で閉じるよう再構成。
    - 各辺長は板長(186/95/49)に固定。
    - 各辺の角度は図の向き(軸/斜め)を目標に、閉合(Σベクトル=0)を満たす最小変位で解く。
    - 先頭点は元位置に固定して歩き直す。
    """
    import numpy as np
    from scipy.optimize import minimize
    seg = []  # (board_len, target_angle_rad)
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        L = math.hypot(dx, dy)
        ang = math.degrees(math.atan2(dy, dx)) % 360
        snapped = None
        for a in (0, 90, 180, 270):
            if abs((ang - a + 180) % 360 - 180) < AXIS_SNAP_DEG:
                snapped = a
                break
        tgt = math.radians(snapped) if snapped is not None else math.radians(ang)
        seg.append((_snap_len(L), tgt))
    Ls = np.array([s[0] for s in seg])
    tgt = np.array([s[1] for s in seg])
    P0 = np.array([pts[0][0], pts[0][1]], float)
    # 目標頂点(元の支柱位置) p1..pN
    Vt = np.array([[pts[i + 1][0], pts[i + 1][1]] for i in range(len(seg))], float)

    def verts(th):
        c = P0 + np.cumsum(np.stack([Ls * np.cos(th), Ls * np.sin(th)], 1), 0)
        return c  # p1..pN

    def obj(th):
        # 各頂点を実物位置へ近づける + 角度が目標から離れすぎない正則化
        v = verts(th)
        return float(np.sum((v - Vt) ** 2) + 500.0 * np.sum((th - tgt) ** 2))

    def close_x(th):
        return float(np.sum(Ls * np.cos(th)))

    def close_y(th):
        return float(np.sum(Ls * np.sin(th)))

    res = minimize(obj, tgt.copy(), method="SLSQP",
                   constraints=[{"type": "eq", "fun": close_x},
                                {"type": "eq", "fun": close_y}],
                   options={"maxiter": 1000, "ftol": 1e-9})
    th = res.x
    out = [(float(P0[0]), float(P0[1]))]
    for i in range(len(seg)):
        px, py = out[-1]
        out.append((px + Ls[i] * math.cos(th[i]), py + Ls[i] * math.sin(th[i])))
    return out


# 出力に使う版: v3(支柱ベース, 位置が図に一致)を採用。
# 道路幅を実寸に合わせて調整する(横=140cm, 左の縦=170cm)。
OUTER_B = OUTER
INNER_B = INNER
START_B = START_DIVIDERS

# P枠 (内寸124, 高さ50, ピッチ180)。left, bottom, right, top, color(rgba)
P_BOXES = [
    ("P1", 250, 0, 374, 50, (0.0, 0.6, 0.0, 1)),   # 緑
    ("P2", 430, 0, 554, 50, (0.8, 0.0, 0.0, 1)),   # 赤
    ("P3", 610, 0, 734, 50, (0.0, 0.0, 0.8, 1)),   # 青
]


def wall_link(name, p1, p2, h=WALL_H, t=WALL_T, rgba=(0.9, 0.9, 0.9, 1)):
    """cm の 2点 -> box link (SDF文字列)"""
    x1, y1 = p1[0] * CM, p1[1] * CM
    x2, y2 = p2[0] * CM, p2[1] * CM
    L = math.hypot(x2 - x1, y2 - y1)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    yaw = math.atan2(y2 - y1, x2 - x1)
    r, g, b, a = rgba
    return f"""    <link name="{name}">
      <pose>{cx:.3f} {cy:.3f} {h/2:.3f} 0 0 {yaw:.4f}</pose>
      <collision name="c"><geometry><box><size>{L:.3f} {t} {h}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{L:.3f} {t} {h}</size></box></geometry>
        <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
      </visual>
    </link>"""


def polyline_links(prefix, pts, rgba=(0.9, 0.9, 0.9, 1)):
    out = []
    for i in range(len(pts) - 1):
        out.append(wall_link(f"{prefix}_{i}", pts[i], pts[i + 1], rgba=rgba))
    return out


def pbox_links(name, l, b, r, t, rgba):
    pts = [(l, b), (r, b), (r, t), (l, t), (l, b)]
    return polyline_links(f"box_{name}", pts, rgba=rgba)


def main():
    links = []
    links += polyline_links("outer", OUTER_B, rgba=(0.85, 0.1, 0.1, 1))  # 外周=赤系
    for i, pl in enumerate(INNER_B):
        links += polyline_links(f"inner{i}", pl, rgba=(0.9, 0.9, 0.9, 1))
    for i, pl in enumerate(START_B):
        links += polyline_links(f"start{i}", pl, rgba=(0.95, 0.95, 0.95, 1))
    for (name, l, b, r, t, rgba) in P_BOXES:
        links += pbox_links(name, l, b, r, t, rgba)

    walls = "\n".join(links)
    cx, cy = W * CM / 2, H * CM / 2  # コース中心(地面/光の基準)

    print(f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="minicar_course">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>14 9</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>14 9</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="course_walls">
      <static>true</static>
{walls}
    </model>
  </world>
</sdf>""")


if __name__ == "__main__":
    main()

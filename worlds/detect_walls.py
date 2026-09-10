#!/usr/bin/env python3
"""
PDF24pレンダリング画像から、支柱(黄丸)間の赤/白壁を色判定で自動抽出し traced.json を出力。
generate_course_traced.py がこれを読む。
"""
import json
import numpy as np
from PIL import Image
from scipy import ndimage

IMG = "/tmp/claude-1000/-home-ryosuke-Docker/9cb06b1f-9456-4ae7-b868-049a8116a0b2/scratchpad/p24-24.png"
OUT = "traced.json"
THIRD_POST_PX = 12      # 線分内部にこの距離以内の別支柱があれば非隣接として除外(旧25→12)
MAX_SEG_PX = 1100
COLOR_FRAC = 0.45


def detect_posts(a):
    R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    mask = (R > 220) & (G > 150) & (G < 210) & (B < 120)
    lbl, n = ndimage.label(mask)
    posts = []
    for i in range(1, n + 1):
        yy, xx = np.where(lbl == i)
        if 200 < len(xx) < 3000 and xx.mean() > 1080:
            posts.append((xx.mean(), yy.mean()))
    return np.array(posts)


def classify(a, p1, p2, N=25):
    x1, y1 = p1; x2, y2 = p2
    red = white = 0
    for t in np.linspace(0.15, 0.85, N):
        x = int(round(x1 + (x2 - x1) * t)); y = int(round(y1 + (y2 - y1) * t))
        best = "o"
        for dy in (-2, 0, 2):
            for dx in (-2, 0, 2):
                r, g, b = a[y + dy, x + dx]
                if r > 150 and g < 120 and b < 120 and r - g > 60:
                    best = "red"
                elif r > 240 and g > 240 and b > 240 and best != "red":
                    best = "white"
        if best == "red":
            red += 1
        elif best == "white":
            white += 1
    return red / N, white / N


def third_post_near(p1, p2, others):
    x1, y1 = p1; x2, y2 = p2
    L2 = (x2 - x1) ** 2 + (y2 - y1) ** 2 + 1e-9
    for q in others:
        t = ((q[0] - x1) * (x2 - x1) + (q[1] - y1) * (y2 - y1)) / L2
        if 0.12 < t < 0.88:
            px, py = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            if np.hypot(q[0] - px, q[1] - py) < THIRD_POST_PX:
                return True
    return False


def main():
    a = np.asarray(Image.open(IMG).convert("RGB")).astype(int)
    posts = detect_posts(a)
    walls = []
    for i in range(len(posts)):
        for j in range(i + 1, len(posts)):
            if np.hypot(*(posts[i] - posts[j])) > MAX_SEG_PX:
                continue
            others = [posts[k] for k in range(len(posts)) if k != i and k != j]
            if third_post_near(posts[i], posts[j], others):
                continue
            rf, wf = classify(a, posts[i], posts[j])
            if rf > COLOR_FRAC:
                walls.append((i, j, "red"))
            elif wf > COLOR_FRAC:
                walls.append((i, j, "white"))
    json.dump({"posts": posts.tolist(), "walls": walls}, open(OUT, "w"))
    print(f"posts={len(posts)} walls={len(walls)} "
          f"(red={sum(1 for w in walls if w[2]=='red')}, "
          f"white={sum(1 for w in walls if w[2]=='white')})")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
RealSense D435 RGB FOV vs Depth FOV 시각화

- 왼쪽: 위에서 본 모습 (x: 좌우, z: 앞 거리)
- 오른쪽: 거리 Z에서 정면으로 본 모습 (카메라 시점)
- 색상: depth만 = 파랑, RGB만 = 빨강, 겹치는 부분 = 보라
- 아래 슬라이더로 Z 변경

실행: python3 realsense_fov_viz.py
의존성: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import Slider

# ---------------- D435 사양 (datasheet 기준, 실제 장비에 맞게 수정) ----------------
DEPTH_HFOV = 87.0   # deg
DEPTH_VFOV = 58.0   # deg
RGB_HFOV = 69.0     # deg
RGB_VFOV = 42.0     # deg
BASELINE = 0.050    # m, 왼쪽 IR ~ 오른쪽 IR 거리
# 왼쪽 IR(depth 원점) 기준 RGB 카메라 가로 위치 [m]. 카메라가 바라보는 방향 기준 + = 오른쪽.
# D435: 오른쪽 IR은 +50mm, RGB는 왼쪽 IR 바깥쪽 약 15mm -> -0.015
# 장비별 실측값: `rs-enumerate-devices -c` 의 Extrinsic from "Depth" To "Color" Translation x 값(≈+0.015)의 부호 반대
RGB_X = -0.015
MIN_Z = 0.28        # m, 최소 측정 거리 (최대 해상도 기준)
Z_MAX = 3.0         # m, 그림 최대 거리
Z_INIT = 0.5        # m, 슬라이더 초기값

C_DEPTH = "#4C8BF5"
C_RGB = "#E5484D"
C_OVERLAP = "#8E4EC6"
ALPHA = 0.35
ALPHA_OV = 0.65

tD = np.tan(np.radians(DEPTH_HFOV / 2))
tDv = np.tan(np.radians(DEPTH_VFOV / 2))
tR = np.tan(np.radians(RGB_HFOV / 2))
tRv = np.tan(np.radians(RGB_VFOV / 2))


def depth_x(z):
    """거리 z에서 depth가 나오는 가로 구간 [l, r] (왼쪽 IR 원점). 없으면 None."""
    left = BASELINE - z * tD   # 오른쪽 IR 시야의 왼쪽 끝
    right = z * tD             # 왼쪽 IR 시야의 오른쪽 끝
    return (left, right) if right > left else None


def rgb_x(z):
    return RGB_X - z * tR, RGB_X + z * tR


def overlap(a, b):
    if a is None or b is None:
        return None
    lo, hi = max(a[0], b[0]), min(a[1], b[1])
    return (lo, hi) if hi > lo else None


def depth_fov_deg(z):
    """발제 공식: HFOV/2 + atan(tan(HFOV/2) - B/Z)"""
    return DEPTH_HFOV / 2 + np.degrees(np.arctan(tD - BASELINE / z))


def fill_region(ax, zs, intervals, color, label, alpha=ALPHA):
    """구간 리스트를 위에서 본 영역으로 채움 (x축=좌우, y축=거리)."""
    lo = np.array([iv[0] if iv else np.nan for iv in intervals])
    hi = np.array([iv[1] if iv else np.nan for iv in intervals])
    ax.fill_betweenx(zs, lo, hi, where=~np.isnan(lo), color=color, alpha=alpha,
                     lw=0, label=label)


def split(a, b):
    """a 구간에서 b와 겹치는 부분을 뺀 나머지 (최대 2조각)."""
    if a is None:
        return []
    ov = overlap(a, b)
    if ov is None:
        return [a]
    parts = []
    if ov[0] > a[0]:
        parts.append((a[0], ov[0]))
    if a[1] > ov[1]:
        parts.append((ov[1], a[1]))
    return parts


# ---------------- 그림 ----------------
fig, (ax_top, ax_front) = plt.subplots(1, 2, figsize=(13, 6.5))
plt.subplots_adjust(bottom=0.18, wspace=0.25)

# 위에서 본 모습 (정적 영역)
zs = np.linspace(1e-3, Z_MAX, 600)
D = [depth_x(z) for z in zs]
R = [rgb_x(z) for z in zs]
O = [overlap(d, r) for d, r in zip(D, R)]
D_only = [split(d, r) for d, r in zip(D, R)]
R_only = [split(r, d) for d, r in zip(D, R)]

for k in range(2):
    fill_region(ax_top, zs, [p[k] if len(p) > k else None for p in D_only],
                C_DEPTH, "depth only" if k == 0 else None)
    fill_region(ax_top, zs, [p[k] if len(p) > k else None for p in R_only],
                C_RGB, "RGB only" if k == 0 else None)
fill_region(ax_top, zs, O, C_OVERLAP, "RGB ∩ depth", alpha=ALPHA_OV)

ax_top.axhspan(0, MIN_Z, color="gray", alpha=0.15, label=f"min Z ({MIN_Z} m)")
ax_top.plot([0, BASELINE], [0, 0], "k-", lw=4)
ax_top.plot(0, 0, "ko", ms=4)
ax_top.plot(BASELINE, 0, "ko", ms=4)
ax_top.plot(RGB_X, 0, "o", color=C_RGB, ms=5)
z_line = ax_top.axhline(Z_INIT, color="k", lw=1, ls="--")
lim = Z_MAX * tD
ax_top.set_xlim(-lim, lim + BASELINE)
ax_top.set_ylim(0, Z_MAX)
ax_top.set_xlabel("x [m] (left IR origin, + = right)")
ax_top.set_ylabel("z [m] (forward)")
ax_top.set_title("Top view")
ax_top.legend(loc="upper left", fontsize=9)
ax_top.grid(alpha=0.3)

# 정면 모습 (슬라이더로 갱신)
info = ax_front.text(0.02, 0.98, "", transform=ax_front.transAxes, va="top",
                     fontsize=10, family="monospace", zorder=10,
                     bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.9))


def draw_front(Z):
    for p in list(ax_front.patches):
        p.remove()
    d = depth_x(Z)
    r = rgb_x(Z)
    hd, hr = Z * tDv, Z * tRv
    if d:
        ax_front.add_patch(Rectangle((d[0], -hd), d[1] - d[0], 2 * hd,
                                     color=C_DEPTH, alpha=ALPHA, lw=0))
    ax_front.add_patch(Rectangle((r[0], -hr), r[1] - r[0], 2 * hr,
                                 color=C_RGB, alpha=ALPHA, lw=0))
    ov = overlap(d, r)
    if ov:
        hv = min(hd, hr)
        ax_front.add_patch(Rectangle((ov[0], -hv), ov[1] - ov[0], 2 * hv,
                                     color=C_OVERLAP, alpha=ALPHA_OV, lw=0))
    # 왼쪽 IR만 보는 띠 (depth 없음) 테두리
    ax_front.add_patch(Rectangle((-Z * tD, -hd), (d[0] if d else Z * tD) + Z * tD, 2 * hd,
                                 fill=False, ec="k", ls=":", lw=1))

    w = max(Z * tD, abs(r[0]), r[1]) * 1.1
    ax_front.set_xlim(-w, w + BASELINE)
    ax_front.set_ylim(-w * 0.75, w * 0.75)

    covered = ov is not None and abs(ov[0] - r[0]) < 1e-9 and abs(ov[1] - r[1]) < 1e-9
    info.set_text(
        f"Z = {Z:.2f} m\n"
        f"depth HFOV(Z) = {depth_fov_deg(Z):5.1f} deg\n"
        f"RGB   HFOV    = {RGB_HFOV:5.1f} deg\n"
        f"depth width   = {(d[1]-d[0])*100 if d else 0:5.1f} cm\n"
        f"RGB width     = {(r[1]-r[0])*100:5.1f} cm\n"
        f"RGB fully covered by depth: {'yes' if covered else 'no'}"
        + ("\n(Z < min Z: depth invalid)" if Z < MIN_Z else "")
    )
    z_line.set_ydata([Z, Z])
    fig.canvas.draw_idle()


ax_front.set_aspect("equal")
ax_front.set_xlabel("x [m]")
ax_front.set_ylabel("y [m]")
ax_front.set_title("Front view at Z (dotted = left-IR view, no depth outside blue/purple)")
ax_front.grid(alpha=0.3)

ax_slider = plt.axes([0.15, 0.06, 0.7, 0.03])
slider = Slider(ax_slider, "Z [m]", 0.05, Z_MAX, valinit=Z_INIT, valstep=0.01)
slider.on_changed(draw_front)
draw_front(Z_INIT)

plt.show()
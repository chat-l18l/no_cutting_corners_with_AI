#!/usr/bin/env python3
"""
make_assets — genereer placeholder-afbeeldingen voor de playground

Tekent per voertuigtype een schematisch bovenaanzicht (matplotlib) en
schrijft PNG's naar tools/viz/assets/<modelnaam>.png. De about-box en het
Voertuig-info-paneel laden deze via hello_imgui's assets-mechanisme.

Vervang de PNG's gerust door echte foto's/renders met dezelfde naam
(fase D van docs/multi_vehicle_plan.md); dit script is alleen de
regenereerbare placeholder-bron.

Draaien:  .venv/bin/python tools/viz/make_assets.py
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches, transforms

ASSETS = Path(__file__).parent / "assets"

BODY = "#3a6ea5"
WHEEL = "#222222"
ACCENT = "#c23b22"


def _wheel(ax, x, y, angle_deg=0.0, length=0.26, width=0.10):
    w = patches.Rectangle((-length / 2, -width / 2), length, width,
                          facecolor=WHEEL, edgecolor="none", zorder=3)
    tr = (transforms.Affine2D().rotate_deg(angle_deg).translate(x, y)
          + ax.transData)
    w.set_transform(tr)
    ax.add_patch(w)


def _arrow(ax, x0=0.62, dx=0.30):
    ax.annotate("", xy=(x0 + dx, 0), xytext=(x0, 0),
                arrowprops=dict(arrowstyle="-|>", lw=2.5, color=ACCENT))


def _finish(ax, title, fname):
    ax.set_xlim(-1.05, 1.05)
    ax.set_ylim(-0.8, 0.8)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=13)
    ax.figure.savefig(ASSETS / fname, dpi=100, bbox_inches="tight",
                      facecolor="white")
    plt.close(ax.figure)
    print(f"geschreven: {ASSETS / fname}")


def draw_ackermann():
    _, ax = plt.subplots(figsize=(5.6, 4.2))
    ax.add_patch(patches.FancyBboxPatch(
        (-0.5, -0.30), 1.0, 0.60, boxstyle="round,pad=0.03",
        facecolor=BODY, edgecolor="black", zorder=2))
    _wheel(ax, -0.38, +0.40)                 # achterwielen recht
    _wheel(ax, -0.38, -0.40)
    _wheel(ax, +0.38, +0.40, angle_deg=22)   # voorwielen gestuurd (Ackermann)
    _wheel(ax, +0.38, -0.40, angle_deg=18)   # binnenwiel iets schuiner
    _arrow(ax)
    _finish(ax, "Ackermann-quad — stuur op vooras, aandrijving achter",
            "ackermann.png")


def draw_skid_steer():
    _, ax = plt.subplots(figsize=(5.6, 4.2))
    ax.add_patch(patches.FancyBboxPatch(
        (-0.45, -0.28), 0.90, 0.56, boxstyle="round,pad=0.03",
        facecolor=BODY, edgecolor="black", zorder=2))
    for sx in (-0.30, 0.30):                 # 4 vaste, brede wielen
        for sy in (0.38, -0.38):
            _wheel(ax, sx, sy, length=0.32, width=0.14)
    _arrow(ax)
    # draaipijl: kan op de plaats draaien
    ax.add_patch(patches.Arc((0, 0), 0.5, 0.5, theta1=210, theta2=120,
                             lw=2.0, color=ACCENT, zorder=4))
    ax.annotate("", xy=(0.24, 0.11), xytext=(0.22, -0.02),
                arrowprops=dict(arrowstyle="-|>", lw=2.0, color=ACCENT))
    _finish(ax, "Skid-steer (Jackal-klasse) — 4 vaste wielen, schuurt in de bocht",
            "skid_steer.png")


def draw_diff_drive():
    _, ax = plt.subplots(figsize=(5.6, 4.2))
    ax.add_patch(patches.Circle((0, 0), 0.42, facecolor=BODY,
                                edgecolor="black", zorder=2))
    _wheel(ax, 0.0, +0.44, length=0.28, width=0.10)
    _wheel(ax, 0.0, -0.44, length=0.28, width=0.10)
    ax.add_patch(patches.Circle((0.30, 0.0), 0.05, facecolor=WHEEL,
                                zorder=3))   # zwenkwiel
    _arrow(ax)
    _finish(ax, "Differential drive (TurtleBot-klasse) — 2 wielen + zwenkwiel",
            "diff_drive.png")


def draw_omni3():
    _, ax = plt.subplots(figsize=(5.6, 4.2))
    ax.add_patch(patches.Circle((0, 0), 0.42, facecolor=BODY,
                                edgecolor="black", zorder=2))
    import math
    for k in range(3):                       # omniwielen op 0°/120°/240°,
        a = math.radians(90 + 120 * k)       # tangentieel gemonteerd
        x, y = 0.46 * math.cos(a), 0.46 * math.sin(a)
        _wheel(ax, x, y, angle_deg=90 + 120 * k + 90, length=0.26, width=0.11)
    _arrow(ax)
    ax.annotate("", xy=(0.0, 0.70), xytext=(0.0, 0.52),
                arrowprops=dict(arrowstyle="-|>", lw=2.0, color=ACCENT))
    _finish(ax, "3-wiel holonomic (omnibot) — omniwielen op 120°",
            "omni3.png")


def main():
    ASSETS.mkdir(exist_ok=True)
    draw_ackermann()
    draw_skid_steer()
    draw_diff_drive()
    draw_omni3()


if __name__ == "__main__":
    main()

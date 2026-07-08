"""Control view in IMAGE-SPACE — trajectories in pixels over the clean frame.

ANALYSIS ONLY. Draws the accumulated tracking trajectories in PIXEL coordinates
(no homography), using the SAME reference point (ground contact) and the SAME
filter/cmap/iteration order as the birdseye, for a 1:1 comparison: the color of each
track matches between this figure and the birdseye one.

Purpose: separate what is tracking noise/sweep (already present in the image) from
what is error that the homography ADDS (appears only when projecting to the world). It does NOT touch any
homography.
"""

import csv
import numpy as np
import pandas as pd
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from digital_shadow.geometry import add_ground_contact

TRACKS = "verify_out/tracks_rusia.csv"
FRAME = "verify_out/homography_improved/clean_frame_2541.png"
CPTS = "verify_out/homography_improved/control_points_improved.csv"
OUT = "verify_out/homography_improved/camera_trajectories.png"
MIN_OBS = 25  # same filter as project_tracks_birdseye.py


def load_cpts():
    pts = []
    with open(CPTS) as f:
        for r in csv.DictReader(f):
            if r["available"] == "True":
                pts.append((r["id"], float(r["pixel_x"]), float(r["pixel_y"])))
    return pts


def main():
    df = pd.read_csv(TRACKS)
    add_ground_contact(df)                    # gx,gy = ground contact (px); see module

    img = cv2.cvtColor(cv2.imread(FRAME), cv2.COLOR_BGR2RGB)
    Hh, Ww = img.shape[:2]

    fig, ax = plt.subplots(figsize=(16, 8.6))
    ax.imshow(img, extent=(0, Ww, Hh, 0), zorder=0)  # origin top-left (pixels)

    cmap = plt.get_cmap("tab20")
    k = 0
    for tid, g in df.groupby("track_id"):
        if len(g) < MIN_OBS:
            continue
        g = g.sort_values("frame_index")
        ax.plot(g["gx"], g["gy"], color=cmap(k % 20), lw=0.8, alpha=0.75, zorder=2)
        k += 1

    # control points (their pixel) to place them relative to the trajectories
    cpts = load_cpts()
    for pid, px, py in cpts:
        ax.plot(px, py, "+", color="lime", ms=12, mew=2.2, zorder=4)
        ax.annotate(pid, (px, py), textcoords="offset points", xytext=(5, 4),
                    fontsize=6.5, color="lime", zorder=5)

    ax.set_xlim(0, Ww); ax.set_ylim(Hh, 0)
    ax.set_title(f"Espacio-IMAGEN: trayectorias (contacto-suelo) en pixeles sobre frame "
                 f"limpio (2541)\n(tracks dibujados={k}; mismo filtro/cmap que el birdseye; "
                 f"+ verde = {len(cpts)} puntos de control)", fontsize=10)
    ax.set_xlabel("u (px)"); ax.set_ylabel("v (px)")
    fig.tight_layout()
    fig.savefig(OUT, dpi=120)
    print(f"Guardado {OUT}  (tracks dibujados={k}, total={df['track_id'].nunique()})")


if __name__ == "__main__":
    main()

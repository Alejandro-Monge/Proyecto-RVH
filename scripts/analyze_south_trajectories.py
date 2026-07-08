"""Targeted analysis of the SOUTH approach — trajectory plotting.

ANALYSIS ONLY. Not a production module. Derives each vehicle's reference point in
POST-PROCESSING from the already-produced tracking CSV
(verify_out/tracks_rusia.csv); does NOT re-track.

Generates, over the first frame as background:
  - full scene with point = box CENTER (cx, cy)
  - full scene with point = GROUND CONTACT (center of bottom edge)
  - enlarged crop of the SOUTH APPROACH (top-left mouth) in both variants

The ground contact is ((x1+x2)/2, y2): same plane (asphalt) for all
vehicles, removing the height parallax of the center. Applied to ALL equally.
"""

import sys
import numpy as np
import pandas as pd
import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from digital_shadow.geometry import add_ground_contact, GROUND_X, GROUND_Y

CSV = "verify_out/tracks_rusia.csv"
BG = "verify_out/first_frame.png"
OUT = "verify_out/south_analysis"

# Crop of the SOUTH approach (top-left mouth): x in [x0,x1], y in [y0,y1].
# Covers the south entry lanes near the horizon/vanishing point.
SOUTH_CROP = dict(x0=0, x1=560, y0=40, y1=320)


def load_points():
    df = pd.read_csv(CSV)
    # Reference point: center (centroid, visual comparison only) vs ground contact
    # (gx,gy = the system's reference point, via the single module).
    df["center_x"] = df["cx"]
    df["center_y"] = df["cy"]
    add_ground_contact(df)   # adds GROUND_X="gx", GROUND_Y="gy"
    return df


def draw(df, xcol, ycol, title, out_path, crop=None, scale=1.0):
    bg = cv2.cvtColor(cv2.imread(BG), cv2.COLOR_BGR2RGB)
    H, W = bg.shape[:2]

    # Figure size at full resolution (1 px = 1 px), no rescaling.
    # `scale` ENLARGES the crop (does not rescale the video: only enlarges the drawing
    # canvas so the lanes can be told apart by eye).
    dpi = 100
    if crop is None:
        fig_w, fig_h = W / dpi, H / dpi
    else:
        fig_w, fig_h = ((crop["x1"] - crop["x0"]) * scale / dpi,
                        (crop["y1"] - crop["y0"]) * scale / dpi)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)
    ax.imshow(bg, extent=[0, W, H, 0])

    # One polyline per track, color by id (cycles through a palette).
    cmap = plt.get_cmap("tab20")
    n_tracks = 0
    for k, (tid, g) in enumerate(df.groupby("track_id")):
        g = g.sort_values("frame_index")
        if len(g) < 2:
            continue
        pts = g[[xcol, ycol]].to_numpy()
        segs = np.stack([pts[:-1], pts[1:]], axis=1)
        lw = 0.8 * (scale if crop is not None else 1.0)
        lc = LineCollection(segs, colors=[cmap(k % 20)], linewidths=lw, alpha=0.7)
        ax.add_collection(lc)
        n_tracks += 1

    if crop is None:
        ax.set_xlim(0, W)
        ax.set_ylim(H, 0)
    else:
        ax.set_xlim(crop["x0"], crop["x1"])
        ax.set_ylim(crop["y1"], crop["y0"])
    ax.set_title(f"{title}  (n_tracks={n_tracks})", fontsize=8)
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout(pad=0.5)
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {out_path}  (n_tracks dibujados={n_tracks})")


def main():
    import os

    os.makedirs(OUT, exist_ok=True)
    df = load_points()
    print(f"CSV: {len(df)} observaciones, {df['track_id'].nunique()} tracks")

    print("Escena completa:")
    draw(df, "center_x", "center_y", "CENTRO de la caja (cx,cy)",
         f"{OUT}/full_center.png")
    draw(df, GROUND_X, GROUND_Y, "CONTACTO SUELO (centro borde inferior)",
         f"{OUT}/full_ground.png")

    print("Recorte brazo SUR (ampliado 3x):")
    draw(df, "center_x", "center_y", "SUR — CENTRO de la caja",
         f"{OUT}/south_center.png", crop=SOUTH_CROP, scale=3.0)
    draw(df, GROUND_X, GROUND_Y, "SUR — CONTACTO SUELO",
         f"{OUT}/south_ground.png", crop=SOUTH_CROP, scale=3.0)


if __name__ == "__main__":
    main()

"""Visual sanity of the preliminary homography — projection of trajectories to the ground.

ANALYSIS ONLY. Projects the GROUND-CONTACT points of the tracking-CSV trajectories
to the ground plane (meters) with the preliminary homography, and overlays them
on the REAL geometry of the SUMO net (lane centers of each approach). It lets one judge
by eye whether the homography produces a sensible shape: straight approaches -> straight, parallel
lanes -> parallel, and where it degrades (far field/SUR).

It measures nothing new: it is the qualitative check that accompanies the per-zone error.
"""

import re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from digital_shadow.geometry import add_ground_contact

H = np.load("verify_out/homography_gcp/homography_img2ground.npy")
TRACKS = "verify_out/tracks_rusia.csv"
NET = "config/sites/rusia/sumo/intersection_rusia.net.xml"
OUT = "verify_out/homography_gcp/birdseye_sanity.png"

# Approaches (net edges) whose lane geometry serves as world reference.
ARM_EDGES = {"E1", "-E1", "E3", "-E3", "E5", "-E5", "E6", "-E6", "E0", "-E0"}


def net_lane_shapes():
    """Extracts the polylines (lane center) from the approach edges of the net."""
    txt = open(NET).read()
    shapes = []
    # <edge id="..."> ... </edge> blocks
    for m in re.finditer(r'<edge id="([^"]+)"[^>]*>(.*?)</edge>', txt, re.S):
        eid, body = m.group(1), m.group(2)
        if eid not in ARM_EDGES:
            continue
        for lm in re.finditer(r'shape="([^"]+)"', body):
            pts = [tuple(map(float, p.split(","))) for p in lm.group(1).split()]
            shapes.append(np.array(pts))
    return shapes


def project(px):
    pts = np.hstack([px, np.ones((len(px), 1))])
    w = (H @ pts.T).T
    return w[:, :2] / w[:, 2:3]


def main():
    df = pd.read_csv(TRACKS)
    add_ground_contact(df)                        # gx,gy = ground contact (px); see module
    world = project(df[["gx", "gy"]].to_numpy())
    df["X"], df["Y"] = world[:, 0], world[:, 1]

    shapes = net_lane_shapes()

    fig, axes = plt.subplots(1, 2, figsize=(20, 9))
    titles = ["Vista de pájaro — extensión completa del net",
              "Zoom al núcleo / campo cercano (donde la homografía es fiable)"]
    # Full view frames the net extent (x[-74,168] y[-15,140] +
    # margin); core zoom around J5 (≈57,74).
    bounds = [dict(xlim=(-90, 180), ylim=(-30, 150)),
              dict(xlim=(20, 130), ylim=(30, 120))]

    cmap = plt.get_cmap("tab20")
    for ax, title, bb in zip(axes, titles, bounds):
        # reference: real net lanes (gray)
        for s in shapes:
            ax.plot(s[:, 0], s[:, 1], color="0.6", lw=2, zorder=1)
        # projected trajectories (color per track), only tracks long enough
        k = 0
        for tid, g in df.groupby("track_id"):
            if len(g) < 25:
                continue
            g = g.sort_values("frame_index")
            ax.plot(g["X"], g["Y"], color=cmap(k % 20), lw=0.7, alpha=0.7, zorder=2)
            k += 1
        ax.set_title(f"{title}\n(tracks dibujados={k}; gris = carriles reales SUMO)",
                     fontsize=10)
        ax.set_xlabel("X mundo (m, +este)"); ax.set_ylabel("Y mundo (m, +norte)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.set_xlim(*bb["xlim"]); ax.set_ylim(*bb["ylim"])

    fig.tight_layout()
    fig.savefig(OUT, dpi=110)
    print(f"Guardado {OUT}  (tracks totales={df['track_id'].nunique()})")


if __name__ == "__main__":
    main()

"""HIGH-RESOLUTION birdseye sanity figure — for strong zoom without pixelation.

RENDER ONLY. Loads an ALREADY-computed homography (does not recalibrate, does not touch .npy/.txt) and a
CSV of control points, and reproduces the SAME figure as compute_homography_improved.py
(full view + core zoom, SUMO lanes in gray, control points in green,
projected trajectories), but at large DPI/canvas. Saves to a separate PNG.

Usage:
  plot_birdseye_hires.py --npy <H.npy> --csv <control_points.csv> --out <fig.png>
                         [--dpi 300] [--w 24] [--h 11] [--label "C (15 pts)"]
"""

import csv
import argparse
import re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from digital_shadow.geometry import add_ground_contact

TRACKS = "verify_out/tracks_rusia.csv"
NET = "config/sites/rusia/sumo/intersection_rusia.net.xml"
ARM_EDGES = {"E1", "-E1", "E3", "-E3", "E5", "-E5", "E6", "-E6", "E0", "-E0"}


def net_lane_shapes():
    txt = open(NET).read()
    shapes = []
    for m in re.finditer(r'<edge id="([^"]+)"[^>]*>(.*?)</edge>', txt, re.S):
        eid, body = m.group(1), m.group(2)
        if eid not in ARM_EDGES:
            continue
        for lm in re.finditer(r'shape="([^"]+)"', body):
            p = [tuple(map(float, q.split(","))) for q in lm.group(1).split()]
            shapes.append(np.array(p))
    return shapes


def load_cpts(path):
    pts = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["available"] == "True":
                pts.append((float(r["world_x"]), float(r["world_y"])))
    return pts


def project(H, px):
    px = np.atleast_2d(np.asarray(px, float))
    h = np.hstack([px, np.ones((len(px), 1))])
    w = (H @ h.T).T
    return w[:, :2] / w[:, 2:3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npy", required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tracks", default=TRACKS,
                    help="CSV de tracks a proyectar (default: baseline). Reconexión: "
                         "verify_out/events/rusia/reconnected/tracks_reconnected.csv")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--w", type=float, default=24.0)
    ap.add_argument("--h", type=float, default=11.0)
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    H = np.load(a.npy)
    cpts = load_cpts(a.csv)
    df = pd.read_csv(a.tracks)
    add_ground_contact(df)                       # gx,gy = ground contact (px); see module
    world = project(H, df[["gx", "gy"]].to_numpy())
    df["X"], df["Y"] = world[:, 0], world[:, 1]
    shapes = net_lane_shapes()

    fig, axes = plt.subplots(1, 2, figsize=(a.w, a.h))
    tag = f" — {a.label}" if a.label else ""
    titles = [f"Vista de pájaro — extensión completa{tag}",
              f"Zoom al núcleo / campo cercano{tag}"]
    bounds = [dict(xlim=(-90, 180), ylim=(-30, 150)),
              dict(xlim=(20, 130), ylim=(30, 120))]
    cmap = plt.get_cmap("tab20")
    for ax, title, bb in zip(axes, titles, bounds):
        for s in shapes:
            ax.plot(s[:, 0], s[:, 1], color="0.6", lw=2, zorder=1)
        for (wx, wy) in cpts:
            ax.plot(wx, wy, "g+", ms=12, mew=2.2, zorder=4)
        k = 0
        for tid, gg in df.groupby("track_id"):
            if len(gg) < 25:
                continue
            gg = gg.sort_values("frame_index")
            ax.plot(gg["X"], gg["Y"], color=cmap(k % 20), lw=0.8, alpha=0.7, zorder=2)
            k += 1
        ax.set_title(f"{title}\n(tracks={k}; gris=carriles SUMO; verde={len(cpts)} "
                     f"puntos de control)", fontsize=11)
        ax.set_xlabel("X mundo (m, +este)"); ax.set_ylabel("Y mundo (m, +norte)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.set_xlim(*bb["xlim"]); ax.set_ylim(*bb["ylim"])
    fig.tight_layout()
    fig.savefig(a.out, dpi=a.dpi)
    px_w, px_h = int(a.w * a.dpi), int(a.h * a.dpi)
    print(f"Guardada HIRES {a.out}  ({a.w}x{a.h} in @ {a.dpi} dpi = ~{px_w}x{px_h} px; "
          f"tracks={k}, puntos de control={len(cpts)})")


if __name__ == "__main__":
    main()

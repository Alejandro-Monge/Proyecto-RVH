"""F3 — Accumulated trajectories in camera space WITH reconnection.

PROVENANCE: **BATCH / OFFLINE — NOT what runs live.** tracks_reconnected.csv is the
batch-global reconnection (complete information, the offline UPPER BOUND against which the causal
method is compared). The live PRODUCTION reconnection is causal-greedy, with a measured and
declared gap (10/15 merges; the missed ones required the future). This figure shows the upper bound;
the WITH/WITHOUT reconnection distinction is never left implicit — the README and this docstring fix it.
Same rules as F2: production exclusion (against reconnected/track_events.csv) and ≥25 obs.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common, trajplot  # noqa: E402

INPUTS = [
    "verify_out/events/rusia/reconnected/tracks_reconnected.csv",
    "verify_out/events/rusia/reconnected/track_events.csv",
    "verify_out/homography_improved/clean_frame_2541.png",
    "verify_out/homography_gcp/control_points.csv",
]


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    common.apply_style()

    img = trajplot.load_frame_rgb()
    cpts = trajplot.load_cpts()
    df = trajplot.load_tracks("verify_out/events/rusia/reconnected/tracks_reconnected.csv",
                              "verify_out/events/rusia/reconnected/track_events.csv")

    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=common.fig_size(common.FULL_WIDTH_MM, aspect=h / w))
    ax.imshow(img, extent=(0, w, h, 0), zorder=0)
    n = trajplot.draw_tracks_camera(ax, df)
    ax.scatter(cpts.pixel_x, cpts.pixel_y, s=18, facecolor=trajplot.C_CPT,
               edgecolor="#0b0b0b", linewidths=0.5, zorder=3)
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")
    handles = [plt.Line2D([], [], marker="o", ls="none", markerfacecolor=trajplot.C_CPT,
                          markeredgecolor="#0b0b0b", markersize=4.5, label="control point")]
    ax.legend(handles=handles, loc="lower left", framealpha=0.9, frameon=True,
              edgecolor="none", facecolor="white", borderpad=0.4)

    common.save_fig(fig, "fig03_trayectorias_camara_reconectadas", stamp,
                    "fig03_trayectorias_camara_reconectadas.py",
                    nota=f"BATCH/OFFLINE (techo; producción en vivo = causal-greedy); {n} "
                         f"trayectorias (≥{trajplot.MIN_OBS} obs, exclusión de producción)")
    print(f"{n} trayectorias reconectadas dibujadas")


if __name__ == "__main__":
    main()

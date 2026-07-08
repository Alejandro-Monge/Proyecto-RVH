"""F2 — Accumulated trajectories in camera space, final tracking, with homography points.

PROVENANCE: CAUSAL, WITHOUT reconnection. tracks_rusia.csv is the output of the ONLINE
production tracker (ByteTrack post-agnostic_nms) — the common input of both paths (causal and batch),
before reconnecting. The production rules are applied: the tracks excluded by zone are dropped
(the same run_core rule) and the ≥25 obs filter of the analysis views (declared).
The control points are the 15 OFFICIAL ones (same as F1), to locate the calibration
relative to the trajectories. Ground contact (gx, gy), like the whole pipeline.
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
    "verify_out/tracks_rusia.csv",
    "verify_out/events/rusia/track_events.csv",
    "verify_out/homography_improved/clean_frame_2541.png",
    "verify_out/homography_gcp/control_points.csv",
]


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    common.apply_style()

    img = trajplot.load_frame_rgb()
    cpts = trajplot.load_cpts()
    df = trajplot.load_tracks("verify_out/tracks_rusia.csv",
                              "verify_out/events/rusia/track_events.csv")

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

    common.save_fig(fig, "fig02_trayectorias_camara", stamp, "fig02_trayectorias_camara.py",
                    nota=f"CAUSAL sin reconexión; {n} tracks (≥{trajplot.MIN_OBS} obs, "
                         "excluidos-por-zona fuera, regla de producción)")
    print(f"{n} tracks dibujados")


if __name__ == "__main__":
    main()

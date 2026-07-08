"""F4 — Trajectories projected onto the SUMO plane (birdseye): official H + network lanes.

PROVENANCE: CAUSAL, WITHOUT reconnection (same tracks as F2), projected to world by the
OFFICIAL homography (C-15pts, the one production uses via site.yaml) over the production network
lanes (pinned md5). It is the "other" homography: image→SUMO plane.
The control points are drawn in their WORLD coordinates (image↔SUMO anchor). Per-track colors
consistent with F2 (same order, same cmap): the same trajectory has the same
color in camera and in plane.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common, trajplot  # noqa: E402

INPUTS = [
    "verify_out/tracks_rusia.csv",
    "verify_out/events/rusia/track_events.csv",
    "verify_out/homography_gcp/homography_img2ground.npy",
    "verify_out/homography_gcp/control_points.csv",
    "config/sites/rusia/sumo/intersection_rusia.net.xml",
]

TRACKS = "verify_out/tracks_rusia.csv"
EVENTS = "verify_out/events/rusia/track_events.csv"
NAME = "fig04_plano_sumo"
NOTA = "CAUSAL sin reconexión; H oficial C-15pts; net de producción"


def main(tracks_csv: str = TRACKS, events_csv: str = EVENTS,
         inputs: list[str] | None = None, name: str = NAME, nota: str = NOTA) -> None:
    stamp = common.check_provenance(inputs or INPUTS)
    common.apply_style()

    H = np.load(trajplot.H_NPY)
    cpts = trajplot.load_cpts()
    df = trajplot.load_tracks(tracks_csv, events_csv)
    shapes = trajplot.net_lane_shapes()
    xlim, ylim = trajplot.world_view_limits(shapes)

    fig, ax = plt.subplots(figsize=common.fig_size(
        common.FULL_WIDTH_MM, aspect=(ylim[1] - ylim[0]) / (xlim[1] - xlim[0])))
    for s in shapes:
        ax.plot(s[:, 0], s[:, 1], color=trajplot.C_NET, lw=2.6, alpha=0.55, zorder=1,
                solid_capstyle="round")
    n = trajplot.draw_tracks_world(ax, df, H)
    ax.scatter(cpts.world_x, cpts.world_y, s=18, facecolor=trajplot.C_CPT,
               edgecolor="#0b0b0b", linewidths=0.5, zorder=3)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")
    trajplot.add_scale_bar(ax, xlim, ylim)
    handles = [plt.Line2D([], [], color=trajplot.C_NET, lw=2.6, alpha=0.55,
                          label="network lane"),
               plt.Line2D([], [], marker="o", ls="none", markerfacecolor=trajplot.C_CPT,
                          markeredgecolor="#0b0b0b", markersize=4.5,
                          label="control point (world)")]
    ax.legend(handles=handles, loc="upper right", framealpha=0.9, frameon=True,
              edgecolor="none", facecolor="white", borderpad=0.4)

    common.save_fig(fig, name, stamp, "fig04_plano_sumo.py",
                    nota=f"{nota}; {n} tracks (≥{trajplot.MIN_OBS} obs, exclusión de producción); "
                         "vista recortada al bbox del net (el campo lejano proyectado queda fuera)")
    print(f"{n} tracks proyectados")


if __name__ == "__main__":
    main()

"""F1 — Homography calibration: clean frame + 15 OFFICIAL control points + stop lines.

PROVENANCE: current production, TIMELESS (no tracks). The 15 points are the current
C-15pts calibration (homography_gcp/control_points.csv, the one site.yaml references), NOT the
candidates in control_points_improved. The stop lines come from zones.yaml (site
config; their endpoints are *-SL-* control points).
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
    "verify_out/homography_improved/clean_frame_2541.png",
    "verify_out/homography_gcp/control_points.csv",
    "config/sites/rusia/zones.yaml",
]


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    common.apply_style()

    img = trajplot.load_frame_rgb()
    cpts = trajplot.load_cpts()
    stops = trajplot.load_stop_lines()

    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=common.fig_size(common.FULL_WIDTH_MM, aspect=h / w))
    ax.imshow(img, extent=(0, w, h, 0), zorder=0)

    for seg in stops.values():
        ax.plot(seg[:, 0], seg[:, 1], color=trajplot.C_STOP, lw=1.4, zorder=2,
                solid_capstyle="round")
    ax.scatter(cpts.pixel_x, cpts.pixel_y, s=26, facecolor=trajplot.C_CPT,
               edgecolor="#0b0b0b", linewidths=0.6, zorder=3)

    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")
    handles = [plt.Line2D([], [], marker="o", ls="none", markerfacecolor=trajplot.C_CPT,
                          markeredgecolor="#0b0b0b", markersize=5,
                          label=f"control points ({len(cpts)})"),
               plt.Line2D([], [], color=trajplot.C_STOP, lw=1.4, label="stop line")]
    ax.legend(handles=handles, loc="lower left", framealpha=0.9, frameon=True,
              edgecolor="none", facecolor="white", borderpad=0.4)

    common.save_fig(fig, "fig01_homografia_calibracion", stamp,
                    "fig01_homografia_calibracion.py",
                    nota="calibración vigente C-15pts (site.yaml→homography_gcp); atemporal")


if __name__ == "__main__":
    main()

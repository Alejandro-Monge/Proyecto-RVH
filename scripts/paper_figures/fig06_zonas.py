"""F6 — The THREE zone layers of the site in one image.

They are THREE DISTINCT SYSTEMS with distinct functions — the caption must not merge them:
  1. EXCLUSION (oeste_parqueadero, zones.yaml): discards contacts BEFORE everything, in the
     events layer (the detector stays "dumb"). Visual channel: TEXTURE (gray hatch).
  2. LANE CLASSIFICATION (lane_bands from zones.yaml): decides the lane AT the stop-line
     crossing (nearest centroid over the transverse coordinate s); feeds the
     shadow (departLane) and the lane metrics. Visual channel: black STROKE on the stop line
     (solid line + dashed decision boundaries, the F8 style).
  3. APPROACH (approach_zones.yaml): counts OCCUPANCY upstream of the stop line (occupancy
     layer, its own lifecycle); lane polygons with declared coverage +
     arm polygon in the background. Visual channel: translucent FILL (F8's c1/c2/c3 palette)
     + white arm outline.

Each layer has its own visual channel (texture / stroke / fill), not just color: they are
distinguished in B/W too.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common, trajplot  # noqa: E402
import derive_track_events as dte  # noqa: E402
from digital_shadow.config.site import load_site_config  # noqa: E402
from digital_shadow.occupancy.zones import load_approach_zones  # noqa: E402

INPUTS = [
    "verify_out/homography_improved/clean_frame_2541.png",
    "config/sites/rusia/zones.yaml",
    "config/sites/rusia/approach_zones.yaml",
    "config/sites/rusia/site.yaml",
    "scripts/derive_track_events.py",     # stamp: stop-line/bands reader (same as production)
]
C_LANE = {1: "#2a78d6", 2: "#1baf7a", 3: "#eda100"}   # validated palette (the one from F8)
C_EXCL = "#6f6f6f"


def frontier_svals(band: dict) -> list[float]:
    """Decision boundaries = midpoints between adjacent centroids over s (like F8)."""
    if not band or band.get("resolution") == "arm_only" or "centroids_s" not in band:
        return []
    vals = sorted(float(v) for v in band["centroids_s"].values())
    return [(a + b) / 2 for a, b in zip(vals, vals[1:])]


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    common.apply_style()

    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    stops = dte.load_stop_lines("rusia")
    bands = dte.load_lane_bands("rusia")
    zones = load_approach_zones("rusia")
    excl = np.array(yaml.safe_load((ROOT / "config/sites/rusia/zones.yaml").read_text())
                    ["exclusion_zones"]["oeste_parqueadero"]["polygon_px"], float)

    img = trajplot.load_frame_rgb()
    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=common.fig_size(common.FULL_WIDTH_MM, aspect=h / w))
    ax.imshow(img, extent=(0, w, h, 0), zorder=0)

    # --- layer 3: APPROACH (translucent fill per lane + arm in white outline) ---
    for arm, az in zones.arms.items():
        pa = az.polygon
        ax.plot(np.r_[pa[:, 0], pa[0, 0]], np.r_[pa[:, 1], pa[0, 1]],
                color="white", lw=0.9, alpha=0.95, zorder=2)
        for k, lz in az.lanes.items():
            pl = lz.polygon
            ax.fill(pl[:, 0], pl[:, 1], facecolor=C_LANE[int(k)], alpha=0.30, zorder=1)
            ax.plot(np.r_[pl[:, 0], pl[0, 0]], np.r_[pl[:, 1], pl[0, 1]],
                    color=C_LANE[int(k)], lw=0.8, alpha=0.9, zorder=2)

    # --- layer 2: CLASSIFICATION (solid stop line + dashed band boundaries, entry+exit) ---
    for arm, seg in stops.items():
        p1 = seg[0]
        u = (seg[1] - seg[0]) / np.linalg.norm(seg[1] - seg[0])
        _p, nh = dte._inward_normal(seg, center)
        ax.plot(seg[:, 0], seg[:, 1], color="#0b0b0b", lw=1.3, alpha=0.9, zorder=3)
        for kind in ("entry", "exit"):
            for sb in frontier_svals(bands.get(kind, {}).get(arm)):
                q = p1 + sb * u
                ax.plot([q[0] - 22 * nh[0], q[0] + 22 * nh[0]],
                        [q[1] - 22 * nh[1], q[1] + 22 * nh[1]],
                        color="#0b0b0b", lw=1.0, ls=(0, (3, 2)), zorder=4)

    # --- layer 1: EXCLUSION (gray hatch, the only one with texture; reinforced so the
    # hatching reads over the gray asphalt of the parking area) ---
    plt.rcParams["hatch.linewidth"] = 1.6
    ax.fill(excl[:, 0], excl[:, 1], facecolor="#8a8a8a", alpha=0.45,
            hatch="////", edgecolor="#2f2f2f", lw=0.0, zorder=1.5)
    ax.plot(np.r_[excl[:, 0], excl[0, 0]], np.r_[excl[:, 1], excl[0, 1]],
            color="#2f2f2f", lw=1.4, ls=(0, (4, 2)), zorder=2)

    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")

    handles = [
        plt.Rectangle((0, 0), 1, 1, facecolor=C_LANE[1], alpha=0.45, edgecolor=C_LANE[1],
                      label="approach zone, lane 1 (occupancy)"),
        plt.Rectangle((0, 0), 1, 1, facecolor=C_LANE[2], alpha=0.45, edgecolor=C_LANE[2],
                      label="approach zone, lane 2"),
        plt.Rectangle((0, 0), 1, 1, facecolor=C_LANE[3], alpha=0.45, edgecolor=C_LANE[3],
                      label="approach zone, lane 3"),
        plt.Line2D([], [], color="white", lw=1.2, label="arm polygon (full depth)"),
        plt.Line2D([], [], color="#0b0b0b", lw=1.3, label="stop line"),
        plt.Line2D([], [], color="#0b0b0b", lw=1.0, ls=(0, (3, 2)),
                   label="lane-band boundary (classification)"),
        plt.Rectangle((0, 0), 1, 1, facecolor="#8a8a8a", alpha=0.55, hatch="////",
                      edgecolor="#2f2f2f", label="exclusion (parking area)"),
    ]
    ax.legend(handles=handles, loc="lower right", framealpha=0.92, frameon=True,
              edgecolor="none", facecolor="white", borderpad=0.4, fontsize=6)

    common.save_fig(fig, "fig06_zonas", stamp, "fig06_zonas.py",
                    nota="TRES sistemas distintos: exclusión descarta en la capa de eventos "
                         "(no en el detector); las bandas clasifican EN la pare (alimentan "
                         "sombra y métricas de carril); las zonas de approach cuentan "
                         "ocupación aguas arriba (capa propia, cobertura declarada por "
                         "carril). Canales visuales: textura / trazo / relleno — legible "
                         "también en B/N.")


if __name__ == "__main__":
    main()

"""F7 — Tracking fragmentation breaks, BASE line (PRE-reconnection), post-agnostic.

PROVENANCE: current production, the state BEFORE reconnecting — that is its value: it shows the
PROBLEM that the reconnection (F3/F5) solves, NOT a failure of the final system. The event
catalog is produced by the project instrument (scripts/measure_fragmentation_baseline.py,
pinned md5) re-run over the current POST-agnostic tracks.

Each point = a death→birth break (gap 0.7–5.1 s, dist ≤250 px), at the midpoint
of the image where it occurs; the tone (light→dark) orders the gap duration.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common, trajplot  # noqa: E402

INPUTS = [
    "verify_out/tracks_rusia.csv",
    "config/sites/rusia/zones.yaml",
    "verify_out/homography_improved/clean_frame_2541.png",
    "scripts/measure_fragmentation_baseline.py",
]
EVENTS = "verify_out/fragmentation_baseline/fragmentation_events_baseline.csv"
# ORDINAL single-tone ramp (gap duration, light→dark)
BINS = [("short (<1.5 s)", lambda s: s < 1.5, "#86b6ef"),
        ("medium (1.5–3 s)", lambda s: 1.5 <= s < 3, "#2a78d6"),
        ("long (≥3 s)", lambda s: s >= 3, "#104281")]


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    # the catalog is a regenerated derivative: its real md5 is stamped (no prior pin)
    stamp[EVENTS] = common.md5_of(EVENTS)
    common.apply_style()

    ev = pd.read_csv(ROOT / EVENTS)
    img = trajplot.load_frame_rgb()
    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=common.fig_size(common.FULL_WIDTH_MM, aspect=h / w))
    ax.imshow(img, extent=(0, w, h, 0), zorder=0)

    handles = []
    for lbl, cond, color in BINS:
        sub = ev[ev.gap_s.map(cond)]
        ax.scatter(sub.mid_x, sub.mid_y, s=34, facecolors="none", edgecolors=color,
                   linewidths=1.3, zorder=3)
        handles.append(plt.Line2D([], [], marker="o", ls="none", markerfacecolor="none",
                                  markeredgecolor=color, markersize=6,
                                  label=f"{lbl}: {len(sub)}"))
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")
    ax.legend(handles=handles, loc="lower left", framealpha=0.9, frameon=True,
              edgecolor="none", facecolor="white", borderpad=0.4,
              title=f"tracking breaks ({len(ev)})", title_fontsize=7)

    common.save_fig(fig, "fig07_fragmentacion_baseline", stamp,
                    "fig07_fragmentacion_baseline.py",
                    nota=f"PRE-reconexión (baseline, el problema que la reconexión resuelve); "
                         f"{len(ev)} rupturas post-agnostic")
    print(f"{len(ev)} rupturas dibujadas")


if __name__ == "__main__":
    main()

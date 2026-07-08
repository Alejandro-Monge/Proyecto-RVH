"""F8 — GT per-lane trails + band boundaries: lane separability made into an image.

Generates SIX figures (one per approach/frame): entry east/north/west/south and exit east/west.
Exit north/south is NOT generated: those mouths have a SINGLE exit lane (lane_map), the
separability figure does not apply (it is not that it is "missing").

PROVENANCE: current production over declared BATCH data — trails = RECONNECTED tracks
(the same source over which the bands were CALIBRATED, zones.yaml) matched to the corrected GT
via the batch matches; bands and transverse coordinate s = the production ones, imported from
derive_track_events (pinned md5), not re-implemented. Each vehicle's point is where the
classifier READS (median at the stop-line crossing, ±3 window).

WHAT THEY SHOW (limits included, not just hits — they are TWO limits of a DIFFERENT nature):
in east/north the bundles separate cleanly and the boundaries cut through empty space. In WEST-entry the
lanes 2 and 3 truly OVERLAP (gap 24.5 px, sd 12–14, overlapping ranges) — a SENSOR
limit, which is why production groups them; the HYPOTHETICAL 2|3 boundary (dotted gray) cuts through the middle
of the mixed cluster. In SOUTH-entry the lanes DO separate in this sample (gaps ~34 px,
sd ≤ 4.3) but n=3–4 per lane does NOT calibrate a classifier — a SAMPLE limit, not a sensor one;
the HYPOTHETICAL boundaries show where a per-lane one that production does NOT apply would fall.

The hypothetical boundaries are derived as the calibration would do it (median of s per GT lane
at the crossing, midpoint between adjacent medians) and are drawn in dotted gray, labeled
"hypothetical" — never in the style of the real ones (dashed black).
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common, trajplot  # noqa: E402

import derive_track_events as dte  # noqa: E402  (production, read-only; pinned md5)
from digital_shadow.config.site import load_site_config  # noqa: E402
from digital_shadow.geometry.ground_contact import add_ground_contact  # noqa: E402

INPUTS = [
    "verify_out/events/rusia/reconnected/tracks_reconnected.csv",
    "verify_out/events/rusia/reconnected/matches.csv",
    "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv",
    "config/sites/rusia/zones.yaml",
    "verify_out/homography_improved/clean_frame_2541.png",
    "scripts/derive_track_events.py",
]
C_LANE = {1: "#2a78d6", 2: "#1baf7a", 3: "#eda100"}   # slots 1–3 of the validated palette
C_HYPO = "#898781"    # hypothetical boundary (not applied): dotted gray, never like the real one
PANELS = [("entry", "este"), ("entry", "norte"), ("entry", "oeste"), ("entry", "sur"),
          ("exit", "este"), ("exit", "oeste")]
MARGIN = 165          # px of slack around the stop line (panel crop)
# HYPOTHETICAL boundaries: NONE in the current configuration (west separated and south enabled,
# both per_lane) — all the production boundaries are real. The mechanism is kept
# in case an approach returns to partial resolution.
HYPO = {}


def frontier_svals(band: dict) -> list[float]:
    """Decision boundaries = midpoints between adjacent centroids over s."""
    if not band or band.get("resolution") == "arm_only" or "centroids_s" not in band:
        return []
    vals = sorted(band["centroids_s"].values())
    return [(a + b) / 2 for a, b in zip(vals, vals[1:])]


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    common.apply_style()

    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    stops = dte.load_stop_lines("rusia")
    bands = dte.load_lane_bands("rusia")
    img = trajplot.load_frame_rgb()
    h, w = img.shape[:2]

    tracks = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/tracks_reconnected.csv")
    add_ground_contact(tracks)
    matches = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv")
    # only the track↔event link; approaches/lanes come from the GT (matches brings its own
    # homonymous columns that would collide in the merge)
    mg = matches[["track_id", "event_id"]].merge(gt, on="event_id")

    for marco, arm in PANELS:
        kind = "entry" if marco == "entry" else "exit"
        arm_col, lane_col = f"{kind}_arm", f"{kind}_lane"
        sel = mg[mg[arm_col] == arm]
        seg = stops[arm]
        p1 = seg[0]
        u = (seg[1] - seg[0]) / np.linalg.norm(seg[1] - seg[0])
        _p, nh = dte._inward_normal(seg, center)
        band = bands.get(kind, {}).get(arm)

        fig, axp = plt.subplots(figsize=common.fig_size(common.COLUMN_WIDTH_MM, aspect=0.72))
        axp.imshow(img, extent=(0, w, h, 0), zorder=0)
        lanes_seen, n_dots = {}, 0
        svals_by_lane: dict[int, list] = {}
        for r in sel.itertuples():
            g = tracks[tracks.track_id == r.track_id].sort_values("frame_index")
            if not len(g):
                continue
            lane = int(getattr(r, lane_col))
            lanes_seen[lane] = lanes_seen.get(lane, 0) + 1
            axp.plot(g.gx, g.gy, color=C_LANE[lane], lw=0.9, alpha=0.8, zorder=2)
            s = dte.lane_cross_coord(g, seg, center, kind=kind)   # where the classifier READS
            if s is not None:
                pt = p1 + s * u
                axp.plot(*pt, marker="o", ms=4.2, mfc=C_LANE[lane], mec="#0b0b0b",
                         mew=0.5, zorder=4)
                svals_by_lane.setdefault(lane, []).append(s)
                n_dots += 1
        axp.plot(seg[:, 0], seg[:, 1], color="#0b0b0b", lw=1.1, alpha=0.85, zorder=3)
        for sb in frontier_svals(band):
            q = p1 + sb * u
            axp.plot([q[0] - 26 * nh[0], q[0] + 26 * nh[0]],
                     [q[1] - 26 * nh[1], q[1] + 26 * nh[1]],
                     color="#0b0b0b", lw=1.0, ls=(0, (3, 2)), zorder=4)
        # HYPOTHETICAL boundaries (not applied): derived as the calibration would do it
        # (median of s per GT lane, midpoint between adjacent medians)
        hypo_pairs = HYPO.get((marco, arm))
        n_hypo = 0
        if hypo_pairs:
            med = {l: float(np.median(v)) for l, v in svals_by_lane.items() if v}
            ml = sorted(med)
            pairs = list(zip(ml, ml[1:])) if hypo_pairs == "all" else \
                [p for p in hypo_pairs if p[0] in med and p[1] in med]
            for a, bpair in pairs:
                sb = (med[a] + med[bpair]) / 2
                q = p1 + sb * u
                xs = [q[0] - 34 * nh[0], q[0] + 34 * nh[0]]
                ys = [q[1] - 34 * nh[1], q[1] + 34 * nh[1]]
                # white underlay: the dotted gray alone does not read over the asphalt
                axp.plot(xs, ys, color="white", lw=2.6, alpha=0.9, zorder=3.5,
                         solid_capstyle="round")
                axp.plot(xs, ys, color=C_HYPO, lw=1.2, ls=(0, (1, 1.4)), zorder=3.6)
                n_hypo += 1

        cx0, cy0 = seg[:, 0].min() - MARGIN, seg[:, 1].min() - MARGIN
        cx1, cy1 = seg[:, 0].max() + MARGIN, seg[:, 1].max() + MARGIN
        axp.set_xlim(max(0, cx0), min(w, cx1))
        axp.set_ylim(min(h, cy1), max(0, cy0))
        axp.axis("off")
        handles = [plt.Line2D([], [], color=C_LANE[l], lw=1.6,
                              label=f"lane {l} (GT, $n$={lanes_seen[l]})")
                   for l in sorted(lanes_seen)]
        if frontier_svals(band):
            handles.append(plt.Line2D([], [], color="#0b0b0b", lw=1.0, ls=(0, (3, 2)),
                                      label="lane-band boundary"))
        if n_hypo:
            handles.append(plt.Line2D([], [], color=C_HYPO, lw=1.0, ls=(0, (1, 1.6)),
                                      label="hypothetical boundary (not applied)"))
        axp.legend(handles=handles, loc="lower left", framealpha=0.9, frameon=True,
                   edgecolor="none", facecolor="white", borderpad=0.35, fontsize=6)

        res = band.get("resolution") if band else "arm_only"
        limite = {("entry", "sur"): "límite de MUESTRA (n=3–4 por carril; separa en la "
                                    "muestra pero no calibra), NO de sensor",
                  ("entry", "oeste"): "límite de SENSOR (solape real 2-3)"}.get((marco, arm), "")
        name = f"fig08_{'entrada' if kind == 'entry' else 'salida'}_{arm}"
        common.save_fig(fig, name, dict(stamp), "fig08_estelas_carril.py",
                        nota=f"BATCH (tracks reconectados, fuente de calibración de bandas); "
                             f"resolución de producción: {res}; {len(sel)} vehículos GT, "
                             f"{n_dots} cruces leídos"
                             + (f"; {n_hypo} fronteras hipotéticas (gris punteado); {limite}"
                                if n_hypo else ""))
        plt.close(fig)
        print(f"{name}: {len(sel)} vehículos, {n_dots} cruces, resolución={res}, hipo={n_hypo}")


if __name__ == "__main__":
    main()

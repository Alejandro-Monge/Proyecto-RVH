"""Calibrates the ENTRY and EXIT LANE BANDS (centroids of the transverse coordinate s on
the stop line) from the vehicles whose lane is KNOWN from GT v2 (not by eye). Prints the
`lane_bands` block (entry/exit) ready to paste into config/sites/<id>/zones.yaml, with N and reason.

REPLICABLE RULE per approach (CURRENT production resolutions = zones.yaml lane_bands.entry;
west and south use per_lane for spatial fidelity of the drawing,
with their bounds declared in zones.yaml):
  ENTRY: east/north/west/south per_lane (west with declared 2|3 overlap; south with sparse sample).
  EXIT : east per_lane (2, error ~14% BEHAVIORAL, within band → individual);
         west per_lane (2, LOO 0/17); north/south per_lane (1 egress lane, trivial).
Entry and exit share the plane (stop_lines) and the u axis; they differ in the crossing direction
(entry first −→+, exit last +→−) and in the centroids. Does NOT classify or write: only computes.
On RECONNECTED tracks.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import add_ground_contact
import derive_track_events as dte

ENTRY_RES = {"este": "per_lane", "norte": "per_lane", "oeste": "per_lane", "sur": "per_lane"}
EXIT_RES = {"este": "per_lane", "norte": "per_lane", "oeste": "per_lane", "sur": "per_lane"}


def collect(matches, gt, by_track, stop, center, side):
    """pts[arm][lane] = [s] for the given side ('entry'/'exit')."""
    arm_col, lane_col = (("entry_arm", "entry_lane") if side == "entry" else ("exit_arm", "exit_lane"))
    pts = {}
    for _, m in matches.iterrows():
        ev = gt.loc[m.event_id]
        arm, lane = getattr(ev, arm_col), int(getattr(ev, lane_col))
        g = by_track.get(int(m.track_id))
        if g is None or arm not in stop:
            continue
        s = dte.lane_cross_coord(g, stop[arm], center, kind=side)
        if s is not None:
            pts.setdefault(arm, {}).setdefault(lane, []).append(s)
    return pts


def emit_arm(arm, res, lanes):
    ns = {L: len(v) for L, v in sorted(lanes.items())}
    if res == "arm_only":
        return f"    {arm}: {{resolution: arm_only}}   # N={ns}"
    if res == "grouped":
        c1 = float(np.median(lanes.get(1, [np.nan])))
        grp = lanes.get(2, []) + lanes.get(3, [])
        c23 = float(np.median(grp)) if grp else float("nan")
        return (f"    {arm}: {{resolution: grouped, centroids_s: {{1: {c1:.1f}, \"2-3\": {c23:.1f}}}}}"
                f"   # N(1)={ns.get(1,0)}, N(2-3)={len(grp)}")
    cents = {L: float(np.median(v)) for L, v in sorted(lanes.items())}
    body = ", ".join(f"{L}: {c:.1f}" for L, c in cents.items())
    return f"    {arm}: {{resolution: per_lane, centroids_s: {{{body}}}}}   # N={ns}"


def main():
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    stop = dte.load_stop_lines("rusia")
    tracks = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/tracks_reconnected.csv")
    add_ground_contact(tracks)
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv").set_index("event_id")
    matches = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    by_track = {t: g.sort_values("frame_index") for t, g in tracks.groupby("track_id")}

    pe = collect(matches, gt, by_track, stop, center, "entry")
    px = collect(matches, gt, by_track, stop, center, "exit")

    print("# --- Bandas de carril (calibradas del GT v2, NO a ojo) ---")
    print("# centroids_s = mediana de s (px, proyección del contacto-suelo sobre la línea de pare).")
    print("# Clasificación = centroide más cercano. entry/exit: mismo plano, distinto sentido y centroides.")
    print("lane_bands:")
    print("  entry:")
    for arm in ["este", "norte", "oeste", "sur"]:
        print(emit_arm(arm, ENTRY_RES[arm], pe.get(arm, {})))
    print("  exit:")
    for arm in ["este", "norte", "oeste", "sur"]:
        print(emit_arm(arm, EXIT_RES[arm], px.get(arm, {})))


if __name__ == "__main__":
    main()

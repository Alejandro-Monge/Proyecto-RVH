"""OBJECTIVE analysis of track continuity (a: survival / b: crossings).

Reads the trajectory CSV and, for each bridged internal gap of a track
(frames lost between consecutive appearances of the SAME track_id), evaluates whether the
reappearance is a clean continuation of the prior motion or a possible
CROSSED IDENTITY:

  extrapolation_error = || pos_reappearance - (pos_before + vel_before * gap) ||

- SMALL error (~ box size) = the track reappears where the previous trajectory
  predicts -> same vehicle, clean bridge (a: ID survives well).
- LARGE error = the box with the old ID appears far from the prediction -> possible
  reassignment to a wrong vehicle (b: crossed identity).

Complements visual inspection; does not replace it (the final judgment is human).

Usage:
  python -m scripts.analyze_track_continuity [--csv verify_out/tracks_rusia.csv] [--min-gap 40]
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict

import numpy as np

from digital_shadow.config import load_site_config


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--csv", default="verify_out/tracks_rusia.csv")
    p.add_argument("--min-gap", type=int, default=40,
                   help="analiza huecos puenteados >= este nº de frames")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    fps = float(load_site_config(args.site).fps_ref)

    # track_id -> list of (frame_index, cx, cy)
    tracks: dict[int, list[tuple[int, float, float]]] = defaultdict(list)
    with open(args.csv) as fh:
        for row in csv.DictReader(fh):
            tracks[int(row["track_id"])].append(
                (int(row["frame_index"]), float(row["cx"]), float(row["cy"]))
            )

    events = []
    for tid, obs in tracks.items():
        obs.sort()
        for i in range(1, len(obs)):
            f0, x0, y0 = obs[i - 1]
            f1, x1, y1 = obs[i]
            gap = f1 - f0 - 1
            if gap < args.min_gap:
                continue
            # previous velocity (px/frame) from the appearance before f0.
            if i >= 2:
                fp, xp, yp = obs[i - 2]
                dt = f0 - fp
                vx, vy = ((x0 - xp) / dt, (y0 - yp) / dt) if dt > 0 else (0.0, 0.0)
            else:
                vx = vy = 0.0
            step = f1 - f0
            ex, ey = x0 + vx * step, y0 + vy * step  # extrapolated position
            err = float(np.hypot(x1 - ex, y1 - ey))
            disp = float(np.hypot(x1 - x0, y1 - y0))
            events.append(dict(tid=tid, f0=f0, f1=f1, gap=gap, disp=disp, err=err))

    events.sort(key=lambda e: e["gap"], reverse=True)
    print("=" * 70)
    print(f"CONTINUIDAD DE TRACKS — huecos puenteados >= {args.min_gap} frames "
          f"(CSV {args.csv})")
    print("=" * 70)
    print(f"  {'track':>6} {'gap_f':>6} {'gap_s':>6} {'desplz':>7} {'err_extrap':>10}  veredicto")
    clean = suspicious = 0
    for e in events:
        # crossing threshold: error larger than the real displacement AND > 60 px
        # (the extrapolation misses badly relative to where it reappears).
        cross = e["err"] > max(60.0, 1.5 * e["disp"])
        verdict = "POSIBLE CRUCE" if cross else "bridge limpio"
        clean += not cross
        suspicious += cross
        print(f"  {e['tid']:>6} {e['gap']:>6} {e['gap']/fps:>6.2f} {e['disp']:>7.0f} "
              f"{e['err']:>10.0f}  {verdict}")
    print("-" * 70)
    print(f"  huecos largos: {len(events)} | bridges limpios: {clean} | "
          f"posibles cruces: {suspicious}")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Ablation: per-event ID survival, for a given tracker CSV.

Reads the event catalog (position+time anchors) and, for a tracker's CSV,
measures per event whether the ID SURVIVES: the track_id nearest to the "before"
anchor (f_before, pos_before) is the SAME as the one nearest to the "after"
anchor (f_after, pos_after), within a radius. Anchored on the scene, NOT on the
baseline's track_id, so it is comparable across trackers.

It is run on each tracker's CSV and the tables are compared. Population A
(ByteTrack fragmentations): how many does the tracker fix? Population B (bridges):
does it keep them (no regression)?

Usage:
  python -m scripts.ablation_survival --csv verify_out/tracks_rusia.csv --label bytetrack
  python -m scripts.ablation_survival --csv verify_out/tracks_rusia_ocsort.csv --label ocsort
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict

import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--catalog", default="verify_out/occlusion/events_catalog.csv")
    p.add_argument("--csv", required=True, help="CSV de trayectorias del tracker a evaluar")
    p.add_argument("--label", required=True, help="etiqueta del tracker (para el reporte)")
    p.add_argument("--radius", type=float, default=60.0,
                   help="radio (px) para asociar un ancla a un track del CSV")
    p.add_argument("--frame-tol", type=int, default=2,
                   help="tolerancia en frames para buscar el ancla")
    return p.parse_args()


def _load_by_frame(path: str) -> dict[int, list[tuple]]:
    by_frame: dict[int, list[tuple]] = defaultdict(list)
    with open(path) as fh:
        for r in csv.DictReader(fh):
            by_frame[int(r["frame_index"])].append(
                (int(r["track_id"]), float(r["cx"]), float(r["cy"]))
            )
    return by_frame


def _id_at(by_frame, frame, x, y, radius, ftol) -> int | None:
    """track_id nearest to (x,y) in [frame-ftol, frame+ftol], within radius."""
    best_id, best_d = None, radius
    for f in range(frame - ftol, frame + ftol + 1):
        for tid, cx, cy in by_frame.get(f, []):
            d = np.hypot(cx - x, cy - y)
            if d <= best_d:
                best_id, best_d = tid, d
    return best_id


def main() -> int:
    args = parse_args()
    by_frame = _load_by_frame(args.csv)

    rows = []
    with open(args.catalog) as fh:
        for e in csv.DictReader(fh):
            id_b = _id_at(by_frame, int(e["f_before"]), float(e["x_before"]),
                          float(e["y_before"]), args.radius, args.frame_tol)
            id_a = _id_at(by_frame, int(e["f_after"]), float(e["x_after"]),
                          float(e["y_after"]), args.radius, args.frame_tol)
            survived = (id_b is not None and id_b == id_a)
            rows.append(dict(eid=e["event_id"], pop=e["pop"], gap=int(e["gap"]),
                             id_b=id_b, id_a=id_a, survived=survived))

    def rate(pop):
        sub = [r for r in rows if r["pop"] == pop]
        s = sum(1 for r in sub if r["survived"])
        return s, len(sub)

    sa, na = rate("A")
    sb, nb = rate("B")
    print("=" * 70)
    print(f"ABLACIÓN — supervivencia de ID  [tracker: {args.label}]  (CSV {args.csv})")
    print("=" * 70)
    print(f"  Población A (fragmentaciones de ByteTrack): {sa}/{na} reconectadas")
    print(f"  Población B (bridges de control)          : {sb}/{nb} mantenidas (no-regresión)")
    print("\n  Detalle población A (las fallas que importan):")
    print(f"    {'ev':>4} {'gap_f':>6} {'id_antes':>9} {'id_despues':>11}  resultado")
    for r in sorted([r for r in rows if r["pop"] == "A"], key=lambda r: r["gap"], reverse=True):
        print(f"    {r['eid']:>4} {r['gap']:>6} {str(r['id_b']):>9} {str(r['id_a']):>11}  "
              f"{'SOBREVIVE' if r['survived'] else 'roto'}")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

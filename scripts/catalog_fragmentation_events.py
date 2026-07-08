"""Catalogs the occlusion events for the ablation, anchored in position+time.

Two populations, both derived from the BASELINE CSV (ByteTrack-140):

  A) FRAGMENTATIONS (the failures that matter): a pair where track A dies -> track B is born
     shortly after and close in space (the vehicle moved while hidden and reappeared with
     a new ID). It is the bus-turn pattern that ByteTrack does NOT re-associate.
  B) BRIDGES (no-regression control): a single track with a long internal gap that
     ByteTrack DID bridge. Used to check that OC-SORT does not break what was already working.

Each event is anchored at (frame_before, pos_before) and (frame_after, pos_after) — points
where the vehicle is visible — so ID survival can be measured with ANY
tracker (the baseline's track_id is not used as a key). Dumps marked frames of the
longest population-A events to visually confirm they are bus turns.

NOTE: fine filtering to the turn zone (south->right) belongs to the zones layer;
here the candidates are listed and confirmed visually. Usage:
  python -m scripts.catalog_fragmentation_events [--csv verify_out/tracks_rusia.csv]
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from digital_shadow.config import load_site_config
from digital_shadow.sources.file_source import OpenCVFrameReader


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--csv", default="verify_out/tracks_rusia.csv",
                   help="CSV baseline (ByteTrack) del que se derivan los eventos")
    p.add_argument("--max-gap", type=int, default=150,
                   help="ventana temporal máx muerte->nacimiento (frames)")
    p.add_argument("--min-gap-a", type=int, default=20,
                   help="hueco mínimo para una fragmentación (frames)")
    p.add_argument("--max-dist", type=float, default=250.0,
                   help="corredor espacial máx entre muerte y nacimiento (px)")
    p.add_argument("--min-gap-b", type=int, default=60,
                   help="hueco interno mínimo para un bridge de control (frames)")
    p.add_argument("--out", default="verify_out/occlusion/events_catalog.csv")
    p.add_argument("--dump-events", type=int, default=6)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    fps = float(site.fps_ref)

    # track_id -> [(frame, cx, cy, class)]
    obs: dict[int, list[tuple]] = defaultdict(list)
    with open(args.csv) as fh:
        for r in csv.DictReader(fh):
            obs[int(r["track_id"])].append(
                (int(r["frame_index"]), float(r["cx"]), float(r["cy"]), int(r["class_id"]))
            )
    for v in obs.values():
        v.sort()

    births = {tid: v[0] for tid, v in obs.items()}   # (f,cx,cy,cls)
    deaths = {tid: v[-1] for tid, v in obs.items()}

    events = []  # dict(pop, f_before, x_before, y_before, f_after, x_after, y_after, gap, cls)

    # (A) fragmentations: death of A -> birth of B close in time and space.
    for a, (fa, xa, ya, ca) in deaths.items():
        best = None
        for b, (fb, xb, yb, cb) in births.items():
            if b == a:
                continue
            gap = fb - fa
            if not (args.min_gap_a <= gap <= args.max_gap):
                continue
            d = float(np.hypot(xb - xa, yb - ya))
            if d > args.max_dist:
                continue
            if best is None or gap < best[0]:
                best = (gap, b, fb, xb, yb, d)
        if best is not None:
            gap, b, fb, xb, yb, d = best
            events.append(dict(pop="A", track_a=a, track_b=b, f_before=fa, x_before=xa,
                               y_before=ya, f_after=fb, x_after=xb, y_after=yb,
                               gap=gap - 1, dist=d, cls=ca))

    # (B) control bridges: long internal gap within a single track.
    for tid, v in obs.items():
        for (f0, x0, y0, c0), (f1, x1, y1, _) in zip(v, v[1:]):
            g = f1 - f0 - 1
            if g >= args.min_gap_b:
                events.append(dict(pop="B", track_a=tid, track_b=tid, f_before=f0,
                                   x_before=x0, y_before=y0, f_after=f1, x_after=x1,
                                   y_after=y1, gap=g, dist=float(np.hypot(x1-x0, y1-y0)),
                                   cls=c0))

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cols = ["event_id", "pop", "track_a", "track_b", "f_before", "x_before", "y_before",
            "f_after", "x_after", "y_after", "gap", "dist", "cls"]
    with open(out_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i, e in enumerate(events):
            w.writerow([i, e["pop"], e["track_a"], e["track_b"], e["f_before"],
                        f"{e['x_before']:.0f}", f"{e['y_before']:.0f}", e["f_after"],
                        f"{e['x_after']:.0f}", f"{e['y_after']:.0f}", e["gap"],
                        f"{e['dist']:.0f}", e["cls"]])

    n_a = sum(1 for e in events if e["pop"] == "A")
    n_b = sum(1 for e in events if e["pop"] == "B")
    print("=" * 70)
    print("CATÁLOGO DE EVENTOS (baseline ByteTrack)")
    print(f"  fragmentaciones (A): {n_a} | bridges de control (B): {n_b}")
    print(f"  catálogo: {out_path.resolve()}")
    print("=" * 70)
    # Listing of the fragmentations (the ones that matter), by descending gap.
    frags = sorted([e for e in events if e["pop"] == "A"], key=lambda e: e["gap"], reverse=True)
    print(f"  {'id':>4} {'A->B':>9} {'gap_f':>6} {'gap_s':>6} {'dist':>6}  corredor")
    for e in frags:
        eid = events.index(e)
        print(f"  {eid:>4} {str(e['track_a'])+'->'+str(e['track_b']):>9} {e['gap']:>6} "
              f"{e['gap']/fps:>6.2f} {e['dist']:>6.0f}  "
              f"({e['x_before']:.0f},{e['y_before']:.0f})->({e['x_after']:.0f},{e['y_after']:.0f})")

    # Dump of marked frames of the longest fragmentations.
    selected = frags[: args.dump_events]
    needed: dict[int, list] = defaultdict(list)
    for e in selected:
        eid = events.index(e)
        for fr, role in [(e["f_before"], "antes"), (e["f_after"], "despues")]:
            needed[fr].append((eid, role, e))
    out_dir = out_path.parent
    reader = OpenCVFrameReader(site.video_path)
    idx = 0
    while reader.grab():
        if idx in needed:
            img = reader.retrieve()
            for eid, role, e in needed[idx]:
                pa = (int(e["x_before"]), int(e["y_before"]))
                pb = (int(e["x_after"]), int(e["y_after"]))
                cv2.line(img, pa, pb, (255, 255, 0), 2)
                cv2.circle(img, pa, 9, (0, 255, 0), 2)
                cv2.circle(img, pb, 9, (0, 0, 255), 2)
                cv2.putText(img, f"frag ev{eid} gap{e['gap']}f {role}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imwrite(str(out_dir / f"frag_ev{eid:02d}_{role}_f{idx:05d}.png"), img)
        idx += 1
    reader.release()
    print(f"\n  frames de las {len(selected)} fragmentaciones más largas en: {out_dir.resolve()}")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Occupancy GT LABELING video — zone-entry detectors + clock + boxes.

Material for the HUMAN ANNOTATOR of the occupancy GT (see
data/ground_truth/rusia/ground_truth_occupancy_rusia.README.md): over the 180 s segment
it burns in (1) the 12 lane-entry DETECTORS — the `entry_side` (v0→v1) edge of
each lane polygon in approach_zones.yaml, the line the human watches being
crossed —, (2) a CLOCK in absolute seconds with milliseconds "NNN.nnn s", top-right
(same base as the crossings GT: t = 0 at the start of the segment, t = frame·12/355) and
(3) the tracking CONTACT POINT ((x1+x2)/2, y2) of each vehicle, without boxes.
ANNOTATION CRITERION: the annotator marks when the vehicle's NOSE
crosses the detector — the white point is the SYSTEM's reference, not the annotator's
(see README and docs/modules/occupancy_zones.md on the nose↔contact offset).

It does NOT re-run detector/tracker: it draws directly from verify_out/tracks_rusia.csv (same
principle as render_tracks_video.py). The detectors are loaded via
`load_approach_zones()` (production loader, fail-fast): if the config mutated, the
render ABORTS instead of burning in a false geometry. It does not touch the causal pipeline.

Usage: python scripts/render_gt_occupancy_video.py [--max-seconds S]
Output: verify_out/inspeccion_approach/gt_occupancy_labeling_tramo.mp4
"""
from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from digital_shadow.config import load_site_config
from digital_shadow.occupancy import load_approach_zones
from digital_shadow.sources.file_source import OpenCVFrameReader

TRACKS = ROOT / "verify_out" / "tracks_rusia.csv"
ZONES_YAML = ROOT / "config" / "sites" / "rusia" / "zones.yaml"
OUT = ROOT / "verify_out" / "inspeccion_approach" / "gt_occupancy_labeling_tramo.mp4"

CLS = {2: "car", 5: "bus", 7: "truck"}
ARM_INITIAL = {"norte": "N", "este": "E", "sur": "S", "oeste": "O"}
LANE_COLOR = {1: (70, 200, 70), 2: (0, 150, 255), 3: (255, 120, 0)}  # BGR c1/c2/c3


def color_for(tid: int):
    """Stable BGR color per track_id (same scheme as render_tracks_video)."""
    hue = int((tid * 47) % 180)
    bgr = cv2.cvtColor(np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def _plate_text(img, text, org, scale, color, thickness=1):
    """Text with a dark plate behind (legibility over any background)."""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    x, y = org
    cv2.rectangle(img, (x - 3, y - th - 3), (x + tw + 3, y + base + 3), (15, 15, 15), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thickness, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--max-seconds", type=float, default=None)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()

    site = load_site_config("rusia")
    fps = float(site.fps_ref)                       # 355/12: t = idx/fps = idx·12/355
    W, H = site.frame_width, site.frame_height

    # zone-entry detectors (fails if approach_zones.yaml does not validate)
    zones = load_approach_zones("rusia")
    detectors = []                                  # (label, seg(2,2), color)
    for arm, az in zones.arms.items():
        for k, lz in az.lanes.items():
            seg = lz.polygon[list(lz.entry_side)]
            detectors.append((f"{ARM_INITIAL[arm]}-c{k}", seg, LANE_COLOR[k]))

    poly_park = np.array(yaml.safe_load(ZONES_YAML.read_text())["exclusion_zones"]
                         ["oeste_parqueadero"]["polygon_px"], dtype=np.int32)

    by_frame = defaultdict(list)
    gc_in = defaultdict(lambda: [0, 0])
    polyf = poly_park.astype(np.float32)
    with TRACKS.open() as fh:
        for r in csv.DictReader(fh):
            tid = int(r["track_id"])
            box = (float(r["x1"]), float(r["y1"]), float(r["x2"]), float(r["y2"]))
            by_frame[int(r["frame_index"])].append((tid, *box, int(r["class_id"])))
            inside = cv2.pointPolygonTest(polyf, ((box[0] + box[2]) / 2, box[3]), False) >= 0
            gc_in[tid][0] += int(inside)
            gc_in[tid][1] += 1
    parking = {tid for tid, (i, n) in gc_in.items() if i / n >= 0.5}

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
    reader = OpenCVFrameReader(site.video_path)
    idx = 0
    while reader.grab():
        if args.max_seconds is not None and idx / fps > args.max_seconds:
            break
        img = reader.retrieve()
        cv2.polylines(img, [poly_park], True, (0, 165, 255), 2)

        # ONLY the contact point (no boxes: what is observed crossing is the track's
        # point, the box distracts). Parked in gray: excluded, not lost.
        for tid, x1, y1, x2, y2, cls in by_frame.get(idx, []):
            gx, gy = int((x1 + x2) / 2), int(y2)
            if tid in parking:
                cv2.circle(img, (gx, gy), 3, (140, 140, 140), -1, cv2.LINE_AA)
            else:
                cv2.circle(img, (gx, gy), 4, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(img, (gx, gy), 4, (0, 0, 0), 1, cv2.LINE_AA)

        # detectors: ONLY the line, no label (the per-lane color is enough; the
        # approach+lane labels live in the README and in the static preview)
        for _label, seg, col in detectors:
            p1, p2 = seg.astype(int)
            cv2.line(img, tuple(p1), tuple(p2), col, 3, cv2.LINE_AA)

        # clock: ABSOLUTE seconds with MILLISECONDS (same base as the GT), top-right;
        # fixed format width -> stable position, no plate jitter
        clock = f"{idx / fps:8.3f} s"
        (cw_, _), _ = cv2.getTextSize(clock, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        _plate_text(img, clock, (W - cw_ - 12, 34), 1.0, (255, 255, 255), 2)
        cv2.putText(img, "GT OCUPACION: anotar cuando la NARIZ del vehiculo cruza el "
                         "detector de su carril (c1 verde / c2 naranja / c3 azul)",
                    (8, H - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(img, "punto blanco = track del sistema (NO la referencia de "
                         "anotacion) | gris = parqueadero (no contar)",
                    (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)
        writer.write(img)
        idx += 1

    writer.release()
    reader.release()
    print(f"video: {out}  ({idx} frames, {idx / fps:.1f}s, {fps:.3f} fps, "
          f"{len(detectors)} detectores)")


if __name__ == "__main__":
    main()

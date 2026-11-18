"""Render of the tracking BASELINE VIDEO — boxes + IDs, DIRECTLY from the CSV.

It does NOT re-run the pipeline (unlike annotate_tracks.py): it draws the boxes and IDs
from verify_out/tracks_rusia.csv over the video frames, so that the video and the count
of scripts/measure_fragmentation_baseline.py are EXACTLY the same set of tracks.

Stable color per track_id (a vehicle that fragments CHANGES color and #id -> the cut is
SEEN). Short ground-contact trail. The parking-lot tracks (WEST exclusion zone) are
drawn in GRAY and the polygon is traced, to distinguish what is excluded from real fragmentation.

Visual validation instrument + baseline against which to compare the reconnection (verify
that it joins the CORRECT ones, not that it just lowers the number by merging distinct vehicles).

Usage: python -m scripts.render_tracks_video [--max-seconds S] [--out verify_out/tracks_baseline.mp4]
"""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
import yaml

from digital_shadow.config import load_site_config
from digital_shadow.sources.file_source import OpenCVFrameReader

ROOT = Path(__file__).resolve().parent.parent
CSV = ROOT / "verify_out" / "tracks_cuenca.csv"
CLS = {2: "car", 5: "bus", 7: "truck"}


def color_for(tid: int):
    """Stable, well-separated BGR color per track_id (hue by id)."""
    hue = int((tid * 47) % 180)
    bgr = cv2.cvtColor(np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="cuenca", help="ID del sitio en config/sites/<site>")
    ap.add_argument("--max-seconds", type=float, default=None)
    ap.add_argument("--tracks", default=str(CSV),
                    help="CSV de tracks a dibujar (default: verify_out/tracks_cuenca.csv)")
    ap.add_argument("--label", default="AUDITORIA YOLO+ByteTrack",
                    help="texto del banner inferior")
    ap.add_argument("--class-from", default=None,
                    help="CSV de eventos (track_id,vehicle_class): usa la CLASE ESTABILIZADA por "
                         "voto (constante por vehículo) en vez de la clase por frame")
    ap.add_argument("--out", default="verify_out/auditoria_yolo.mp4")
    args = ap.parse_args()

    # Stabilized class per track (optional): a constant label per #id.
    stab = {}
    if args.class_from:
        with open(args.class_from) as fh:
            for r in csv.DictReader(fh):
                c = (r.get("vehicle_class") or "").strip()
                if c:
                    stab[int(r["track_id"])] = c

    site = load_site_config(args.site)
    fps = float(site.fps_ref)
    zones_path = ROOT / "config" / "sites" / args.site / "zones.yaml"
    zones_data = yaml.safe_load(zones_path.read_text()) if zones_path.is_file() else {}
    excl_polys = [
        np.array(z["polygon_px"], dtype=np.int32)
        for z in (zones_data.get("exclusion_zones") or {}).values()
        if "polygon_px" in z
    ]
    stop_lines = {
        arm: np.array(seg, dtype=np.int32)
        for arm, seg in (zones_data.get("stop_lines") or {}).items()
    }

    # CSV -> obs per track and per frame
    by_frame = defaultdict(list)   # frame -> [(tid,x1,y1,x2,y2,cls)]
    by_track = defaultdict(list)
    with Path(args.tracks).open() as fh:
        for r in csv.DictReader(fh):
            tid = int(r["track_id"])
            box = (float(r["x1"]), float(r["y1"]), float(r["x2"]), float(r["y2"]))
            by_frame[int(r["frame_index"])].append((tid, *box, int(r["class_id"])))
            by_track[tid].append((box[0]+box[2])/2)  # only for counting; parking below

    # parking-lot set (majority of ground contact inside any exclusion polygon)
    gc_in = defaultdict(lambda: [0, 0])
    if excl_polys:
        polyfs = [p.astype(np.float32) for p in excl_polys]
        for f, rows in by_frame.items():
            for tid, x1, y1, x2, y2, _ in rows:
                pt = ((x1 + x2) / 2, y2)
                inside = any(cv2.pointPolygonTest(pf, pt, False) >= 0 for pf in polyfs)
                gc_in[tid][0] += int(inside)
                gc_in[tid][1] += 1
    parking = {tid for tid, (i, n) in gc_in.items() if n > 0 and i / n >= 0.5}

    reader = OpenCVFrameReader(site.video_path)
    cap = reader._cap
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out), fourcc, fps, (W, H))
    if not writer.isOpened():
        reader.release()
        raise RuntimeError(f"No se pudo inicializar VideoWriter para {out} ({W}x{H})")
    trails = defaultdict(lambda: deque(maxlen=int(2 * fps)))
    max_s = args.max_seconds if args.max_seconds is not None else float(site.t1_s)

    idx = 0
    n_frames = 0
    while reader.grab():
        if max_s is not None and idx / fps > max_s:
            break
        img = reader.retrieve()
        for poly in excl_polys:
            cv2.polylines(img, [poly], True, (0, 165, 255), 2)
        for arm, seg in stop_lines.items():
            cv2.line(img, tuple(seg[0]), tuple(seg[1]), (255, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(img, arm, tuple(seg[0]), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 0), 1, cv2.LINE_AA)

        for tid, x1, y1, x2, y2, cls in by_frame.get(idx, []):
            gx, gy = int((x1 + x2) / 2), int(y2)
            is_park = tid in parking
            col = (150, 150, 150) if is_park else color_for(tid)
            p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(img, p1, p2, col, 2)
            clabel = stab[tid] if tid in stab else CLS.get(cls, cls)  # stabilized class if present
            tag = f"#{tid} {clabel}" + (" [parq]" if is_park else "")
            cv2.putText(img, tag, (int(x1), max(12, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
            if not is_park:
                trails[tid].append((gx, gy))
                pts = trails[tid]
                for a, b in zip(pts, list(pts)[1:]):
                    cv2.line(img, a, b, col, 1, cv2.LINE_AA)

        cv2.putText(img, f"{args.label} | f{idx} t{idx/fps:5.1f}s | "
                    f"gris=parqueadero(excluido)", (8, H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        writer.write(img)
        idx += 1
        n_frames += 1

    writer.release()
    reader.release()
    print(f"video: {out.resolve()}  ({n_frames} frames, {n_frames/fps:.1f}s, {fps:.3f} fps)")


if __name__ == "__main__":
    main()

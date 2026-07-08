"""Re-examination of the HIGH TAIL of the occlusion distribution (long gaps).

`measure_occlusion.py` gave the distribution; this script LOCATES the long gaps
(track, event frames, before/after centroids) and dumps MARKED frames of
each event, to visually inspect whether each long gap is a REAL OCCLUSION
by a bus or a WRONG ASSOCIATION. It turns "occlusions of ~4 s were seen" into a
datum measured on the video.

On each dumped frame it marks: centroid BEFORE the gap (green), centroid AFTER
(red) and the line between them (the corridor along which the vehicle traveled hidden). If
a bus covers that corridor during the gap -> real occlusion.

Usage:
  python -m scripts.inspect_occlusion_tail [--lost-buffer 150] [--min-gap 75] [--dump-events 8]
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from digital_shadow.config import load_detector_config, load_site_config, load_tracker_config
from digital_shadow.perception import YoloDetector
from digital_shadow.sources.file_source import OpenCVFrameReader
from digital_shadow.tracking import ByteTracker

CLASS_NAMES = {2: "car", 5: "bus", 7: "truck"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--lost-buffer", type=int, default=150,
                   help="buffer generoso de medición (frames)")
    p.add_argument("--min-gap", type=int, default=75,
                   help="reporta huecos >= este nº de frames (la cola alta)")
    p.add_argument("--dump-events", type=int, default=8,
                   help="cuántos de los huecos más largos volcar a frames")
    p.add_argument("--out-dir", default="verify_out/occlusion")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()
    trk_cfg = load_tracker_config()
    fps = float(site.fps_ref)

    print("=" * 70)
    print("RE-EXAMEN DE LA COLA DE OCLUSIÓN — huecos largos: ¿real o asociación errónea?")
    print(f"  buffer de medición {args.lost_buffer} frames | reportando huecos >= "
          f"{args.min_gap} frames (~{args.min_gap/fps:.1f} s)")
    print("=" * 70)

    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    detector.warmup(site.frame_height, site.frame_width)
    tracker = ByteTracker(trk_cfg, frame_rate=fps, lost_track_buffer=args.lost_buffer)

    # Pass 1: appearances per track with centroid and class.
    appearances: dict[int, list[tuple]] = defaultdict(list)
    reader = OpenCVFrameReader(site.video_path)
    index = 0
    while reader.grab():
        tracked = tracker.update(detector.detect(reader.retrieve()))
        for t in tracked:
            appearances[t.track_id].append((index, t.cx, t.cy, t.class_id))
        index += 1
    reader.release()

    # Gap events with metadata.
    events = []
    for tid, apps in appearances.items():
        apps.sort()
        for (fa, cxa, cya, cls), (fb, cxb, cyb, _) in zip(apps, apps[1:]):
            gap = fb - fa - 1
            if gap >= args.min_gap:
                disp = float(np.hypot(cxb - cxa, cyb - cya))
                events.append(dict(track_id=tid, fa=fa, fb=fb, gap=gap, cls=cls,
                                   cxa=cxa, cya=cya, cxb=cxb, cyb=cyb, disp=disp))
    events.sort(key=lambda e: e["gap"], reverse=True)

    print(f"\n  huecos >= {args.min_gap} frames: {len(events)}")
    print(f"  {'rank':>4} {'track':>6} {'gap_f':>6} {'gap_s':>6} {'clase':>6} "
          f"{'desplz_px':>9}  corredor (antes->después)")
    for i, e in enumerate(events):
        print(f"  {i:>4} {e['track_id']:>6} {e['gap']:>6} {e['gap']/fps:>6.2f} "
              f"{CLASS_NAMES.get(e['cls'], e['cls']):>6} {e['disp']:>9.0f}  "
              f"({e['cxa']:.0f},{e['cya']:.0f})->({e['cxb']:.0f},{e['cyb']:.0f})")

    # Pass 2: dump marked frames of the N longest gaps.
    selected = events[: args.dump_events]
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # frame_index -> list of (rank, role) to draw.
    needed: dict[int, list] = defaultdict(list)
    for rank, e in enumerate(selected):
        mids = [e["fa"] + (e["gap"] + 1) * k // 3 for k in (1, 2)]  # two frames at the middle
        for fr, role in [(e["fa"], "antes"), (mids[0], "medio1"),
                         (mids[1], "medio2"), (e["fb"], "despues")]:
            needed[fr].append((rank, role, e))

    reader = OpenCVFrameReader(site.video_path)
    index = 0
    while reader.grab():
        if index in needed:
            img = reader.retrieve()
            for rank, role, e in needed[index]:
                pa = (int(e["cxa"]), int(e["cya"]))
                pb = (int(e["cxb"]), int(e["cyb"]))
                cv2.line(img, pa, pb, (255, 255, 0), 2)
                cv2.circle(img, pa, 9, (0, 255, 0), 2)   # before = green
                cv2.circle(img, pb, 9, (0, 0, 255), 2)   # after = red
                cv2.putText(img, f"ev{rank} t{e['track_id']} gap{e['gap']}f {role}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imwrite(str(out_dir / f"ev{rank:02d}_{role}_f{index:05d}.png"), img)
        index += 1
    reader.release()

    print(f"\n  frames marcados de los {len(selected)} huecos más largos en: "
          f"{out_dir.resolve()}")
    print("  (verde=antes, rojo=después, línea=corredor; ¿hay un bus encima?)")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

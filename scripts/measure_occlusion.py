"""Occlusion duration measurement to set `lost_track_buffer` (with evidence).

Method: run detector+tracker over the segment with a GENEROUS buffer (so the
observable gaps are not truncated) and measure, per track, the GAPS that the tracker
bridges — consecutive frames in which the same `track_id` disappeared (occlusion) and
reappeared with the same id. The distribution of those gaps is the occlusion
duration that the system must be able to cross. `lost_track_buffer` is set to
cover the observed high percentile (with margin), not by eye.

Caveat: a buffer that is too generous may bridge wrong associations (two
different vehicles) and inflate gaps; that is why the high percentile is used and
then validated visually on the video annotated with IDs, not the absolute maximum.

No pacing (this is a measurement, not the operational run); reuses OpenCVFrameReader.

Usage:
  python -m scripts.measure_occlusion [--lost-buffer 150]
"""

from __future__ import annotations

import argparse
from collections import defaultdict

import numpy as np

from digital_shadow.config import load_detector_config, load_site_config, load_tracker_config
from digital_shadow.perception import YoloDetector
from digital_shadow.sources.file_source import OpenCVFrameReader
from digital_shadow.tracking import ByteTracker


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--lost-buffer", type=int, default=150,
                   help="buffer GENEROSO para la medición (frames); no es el operativo")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()
    trk_cfg = load_tracker_config()
    fps = float(site.fps_ref)

    print("=" * 70)
    print("MEDICIÓN DE OCLUSIÓN — para fijar lost_track_buffer con evidencia")
    print(f"  buffer de medición (generoso): {args.lost_buffer} frames "
          f"(~{args.lost_buffer / fps:.1f} s); el operativo se decide del resultado")
    print(f"  conf detector: {det_cfg.confidence} | fps_ref: {fps:.3f}")
    print("=" * 70)

    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    detector.warmup(site.frame_height, site.frame_width)
    tracker = ByteTracker(trk_cfg, frame_rate=fps, lost_track_buffer=args.lost_buffer)

    reader = OpenCVFrameReader(site.video_path)
    appearances: dict[int, list[int]] = defaultdict(list)
    index = 0
    while reader.grab():
        image = reader.retrieve()
        tracked = tracker.update(detector.detect(image))
        for t in tracked:
            appearances[t.track_id].append(index)
        index += 1
    reader.release()

    # Gaps bridged per track (frames lost between consecutive appearances).
    gaps: list[int] = []
    for frames in appearances.values():
        frames.sort()
        for a, b in zip(frames, frames[1:]):
            g = b - a - 1
            if g > 0:
                gaps.append(g)

    print("\nRESULTADO")
    print("-" * 70)
    print(f"  frames procesados        : {index}")
    print(f"  tracks confirmados        : {len(appearances)}")
    print(f"  huecos puenteados (n)     : {len(gaps)}")
    if gaps:
        arr = np.array(gaps)
        for label, val in [
            ("máximo", arr.max()),
            ("p99", np.percentile(arr, 99)),
            ("p95", np.percentile(arr, 95)),
            ("p90", np.percentile(arr, 90)),
            ("mediana", np.percentile(arr, 50)),
        ]:
            print(f"  hueco {label:8s}        : {val:6.1f} frames  (~{val / fps:.2f} s)")
        print(f"\n  Sugerencia (cubrir p95 con margen ~1.5x): "
              f"lost_track_buffer ≈ {int(np.ceil(np.percentile(arr, 95) * 1.5))} frames")
    else:
        print("  (sin huecos: ningún track sufrió oclusión puenteada)")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

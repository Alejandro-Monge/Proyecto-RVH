"""Measurement 2 — REAL-TIME: the detector's real drop rate as a consumer.

The detector consumes the source interface in real-time mode (with pacing and
dropping). The longer `detect()` takes per frame, the more the source drops. It reports
the real DROP RATE over the segment: the number that decides real-time
viability and informs the embedded-hardware argument (Jetson).

The detector is warmed up BEFORE opening the source (a dummy inference), so that
the session t0 is not charged with the CUDA init and the dropping reflects the regime.

It traverses the full segment until `end_of_source` (~180 s of wall-clock because it is
real-time). `--max-seconds` bounds the window to iterate.

Usage:
  python -m scripts.run_detector_realtime [--max-seconds S]
"""

from __future__ import annotations

import argparse
import time

from digital_shadow.config import load_detector_config, load_site_config
from digital_shadow.perception import YoloDetector
from digital_shadow.sources import file_source_from_site
from scripts.measure_utils import GpuSampler


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--max-seconds", type=float, default=None,
                   help="acota la ventana de medio (s); sin él, hasta EOF")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()

    print("=" * 70)
    print("REAL-TIME — descarte real del detector como consumidor de la fuente")
    print(f"  modelo     : {det_cfg.weights} | half(FP16)={det_cfg.half} | imgsz={det_cfg.imgsz}")
    print(f"  clases     : {list(site.class_set)}")
    print(f"  total tramo: {site.expected_total_frames} frames "
          f"(fps_ref {float(site.fps_ref):.3f})")
    print(f"  ventana max: {'tramo completo' if args.max_seconds is None else f'{args.max_seconds} s'}")
    print("=" * 70)

    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    print(f"  clases activas -> ids COCO {detector.class_ids}")
    # Warmup BEFORE opening the source: the CUDA init must not count as dropping.
    detector.warmup(site.frame_height, site.frame_width)

    total_detections = 0
    wall_start = time.monotonic()
    with GpuSampler() as gpu:
        with file_source_from_site(site) as source:
            while True:
                frame = source.read()
                if frame.end_of_source:
                    break
                detections = detector.detect(frame.image)
                total_detections += len(detections)
                if args.max_seconds is not None and frame.timestamp_s >= args.max_seconds:
                    break
            delivered = source.delivered_frames
            discarded = source.discarded_frames

    wall_elapsed = time.monotonic() - wall_start
    consumed = delivered + discarded
    discard_rate = 100.0 * discarded / consumed if consumed else 0.0
    gpu_sum = gpu.summary()

    print("\nRESULTADO (REAL-TIME)")
    print("-" * 70)
    print(f"  entregados (procesados)    : {delivered}")
    print(f"  descartados por la fuente  : {discarded}")
    print(f"  consumidos (entr+desc)     : {consumed}")
    print(f"  TASA DE DESCARTE           : {discard_rate:.2f} %")
    print(f"  detecciones totales        : {total_detections}")
    print(f"  fps efectivos (procesados) : {delivered / wall_elapsed:.2f} fps")
    print(f"  tiempo de reloj            : {wall_elapsed:.2f} s")
    if gpu_sum.available and gpu_sum.util_mean is not None:
        print(f"  GPU utilización media/pico : {gpu_sum.util_mean:.0f} / {gpu_sum.util_peak:.0f} %")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

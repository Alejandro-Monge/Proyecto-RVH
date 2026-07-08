"""Measurement 1 — DIAGNOSTIC: raw YOLOv8x fps (no pacing, no dropping).

Processes the video frames at MAXIMUM speed —without the source's real-time
clock— only to measure how many fps YOLO processes on this machine. It is a
HARDWARE DIAGNOSTIC datum, reported AS SUCH: it is NOT the official protocol
run. It decides whether YOLOv8x has a chance of keeping the 29.583 fps pace or whether
a smaller entry model is needed.

Reuses `OpenCVFrameReader` (same grab/retrieve decode) in a clockless loop;
it does NOT use the real-time source (the closed alternative from design Section 8.2 is not reopened).
The warmup window (cold GPU / CUDA init / first inference) is discarded.

Usage:
  python -m scripts.diagnose_detector_fps [--warmup 10] [--max-frames N]
"""

from __future__ import annotations

import argparse
import time

import torch

from digital_shadow.config import load_detector_config, load_site_config
from digital_shadow.perception import YoloDetector
from digital_shadow.sources.file_source import OpenCVFrameReader
from scripts.measure_utils import GpuSampler, summarize_ms


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--warmup", type=int, default=10,
                   help="frames de warmup descartados (GPU fría)")
    p.add_argument("--max-frames", type=int, default=None,
                   help="limita el nº de frames procesados (por defecto, todos)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()

    print("=" * 70)
    print("DIAGNÓSTICO — fps de YOLO en bruto (sin pacing, NO es corrida oficial)")
    print(f"  modelo     : {det_cfg.weights}  | half(FP16)={det_cfg.half} | "
          f"imgsz={det_cfg.imgsz} | conf={det_cfg.confidence} | device={det_cfg.device}")
    print(f"  clases     : {list(site.class_set)}")
    print(f"  warmup     : {args.warmup} frames descartados")
    print("=" * 70)

    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    print(f"  clases activas -> ids COCO {detector.class_ids}")
    # Explicit pre-warmup to pay the CUDA init before the loop.
    detector.warmup(site.frame_height, site.frame_width)

    reader = OpenCVFrameReader(site.video_path)
    times_ms: list[float] = []
    processed = 0
    index = 0

    if torch.cuda.is_available():
        torch.cuda.reset_peak_memory_stats()

    with GpuSampler() as gpu:
        while reader.grab():
            image = reader.retrieve()
            t0 = time.perf_counter()
            detector.detect(image)
            dt_ms = (time.perf_counter() - t0) * 1000.0

            if index >= args.warmup:
                times_ms.append(dt_ms)
            processed += 1
            index += 1
            if args.max_frames is not None and processed >= args.max_frames:
                break
    reader.release()

    timing = summarize_ms(times_ms)
    gpu_sum = gpu.summary()
    torch_mem_mb = (
        torch.cuda.max_memory_allocated() / 1e6 if torch.cuda.is_available() else None
    )

    print("\nRESULTADO (DIAGNÓSTICO)")
    print("-" * 70)
    print(f"  frames procesados (tras warmup): {timing.n}")
    print(f"  fps medio                      : {timing.fps:.2f} fps")
    print(f"  ms/frame medio                 : {timing.mean_ms:.2f} ms")
    print(f"  ms/frame p50 / p95             : {timing.p50_ms:.2f} / {timing.p95_ms:.2f} ms")
    print(f"  fps_ref del video (objetivo)   : {float(site.fps_ref):.3f} fps")
    veredicto = "SÍ sigue el ritmo" if timing.fps >= float(site.fps_ref) else "NO sigue el ritmo"
    print(f"  ¿sigue 29.583 fps?             : {veredicto}")
    if torch_mem_mb is not None:
        print(f"  GPU memoria pico (torch)       : {torch_mem_mb:.0f} MB")
    if gpu_sum.available and gpu_sum.util_mean is not None:
        print(f"  GPU utilización media / pico   : {gpu_sum.util_mean:.0f} / {gpu_sum.util_peak:.0f} %")
        print(f"  GPU memoria usada pico (smi)   : {gpu_sum.mem_peak_mb:.0f} MB")
    else:
        print("  GPU utilización                : (nvidia-smi no disponible)")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

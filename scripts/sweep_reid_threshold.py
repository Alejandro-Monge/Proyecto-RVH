"""Sweep of the re-id similarity threshold (Path D), per OSNet variant.

Reuses the already-implemented Path D layer. Loads detector + OSNet ONCE per model and
does a pass WITHOUT pacing for each threshold (clean benefit, without confusing it with
dropping). Writes a trajectories CSV per (model, threshold); the metrics
(reconnection/crossings/count) are obtained later with ablation_survival and
analyze_track_continuity over those CSVs. The dropping (cost) is measured separately in
real-time, per MODEL (it does not depend on the threshold: embedding extraction happens
the same; the threshold only changes the match decision).

It does NOT change the operational default (re-id stays OFF in config). It trains NOTHING.

Usage:
  python -m scripts.sweep_reid_threshold --model osnet_x1_0_msmt17 --thresholds 0.3,0.4,0.5,0.6,0.7
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from digital_shadow.config import (
    find_project_root,
    load_detector_config,
    load_site_config,
    load_tracker_config,
)
from digital_shadow.perception import YoloDetector
from digital_shadow.sources.file_source import OpenCVFrameReader
from digital_shadow.tracking import ByteTracker
from digital_shadow.tracking.reid.embedding import OsnetEmbeddingModel
from digital_shadow.tracking.reid.reconnector import ReidReconnector

CSV_HEADER = ["timestamp_s", "frame_index", "track_id",
              "x1", "y1", "x2", "y2", "cx", "cy", "class_id", "confidence"]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--model", required=True, help="nombre del peso OSNet (en models/)")
    p.add_argument("--thresholds", default="0.3,0.4,0.5,0.6,0.7")
    p.add_argument("--margin", type=float, default=None,
                   help="override del margen anti-empate (sub-experimento); por defecto, config")
    p.add_argument("--out-dir", default="verify_out/sweep")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()
    trk_cfg = load_tracker_config()
    fps = float(site.fps_ref)
    thresholds = [float(x) for x in args.thresholds.split(",")]
    margin = args.margin if args.margin is not None else trk_cfg.reid.similarity_margin
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"BARRIDO re-id | modelo {args.model} | umbrales {thresholds}")
    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    detector.warmup(site.frame_height, site.frame_width)
    weights = find_project_root() / "models" / f"{args.model}.pt"
    embedding_model = OsnetEmbeddingModel(
        weights, device=trk_cfg.reid.device, half=trk_cfg.reid.half
    ).load()  # loaded ONCE, reused across all thresholds

    for thr in thresholds:
        tracker = ByteTracker(
            trk_cfg, frame_rate=fps, lost_track_buffer=site.lost_track_buffer_frames
        )
        reconnector = ReidReconnector(
            embedding_model,
            similarity_threshold=thr,
            similarity_margin=margin,
            embedding_refresh_frames=trk_cfg.reid.embedding_refresh_frames,
            temporal_window_frames=site.reid_temporal_window_frames,
            spatial_corridor_px=site.reid_spatial_corridor_px,
        )
        out_csv = out_dir / f"tracks_{args.model}_t{thr:.2f}_m{margin:.2f}.csv"
        reader = OpenCVFrameReader(site.video_path)
        idx = 0
        with open(out_csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(CSV_HEADER)
            while reader.grab():
                image = reader.retrieve()
                tracked = reconnector.apply(
                    tracker.update(detector.detect(image)), image, idx
                )
                ts = idx / fps
                for t in tracked:
                    w.writerow([f"{ts:.6f}", idx, t.track_id,
                                f"{t.x1:.2f}", f"{t.y1:.2f}", f"{t.x2:.2f}", f"{t.y2:.2f}",
                                f"{t.cx:.2f}", f"{t.cy:.2f}", t.class_id, f"{t.confidence:.4f}"])
                idx += 1
        reader.release()
        print(f"  umbral {thr:.2f} -> {out_csv}")
    print("BARRIDO COMPLETO")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

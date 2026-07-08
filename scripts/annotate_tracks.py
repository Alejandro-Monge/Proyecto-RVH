"""Visual validation of tracking — video annotated with persistent IDs and trail.

Pass WITHOUT pacing (full coverage -> smooth video) that runs
`detector -> ByteTrack` and draws, with the supervision annotators, the box, the
`#id class` label and the trajectory TRAIL, with a stable color per track_id. So a
human can judge TRACKING QUALITY: does the id hold while the
vehicle crosses? does it survive the occlusion behind a bus or jump to a new id? It is NOT
compared against the ground truth.

Usage:
  python -m scripts.annotate_tracks [--max-seconds S] [--out-dir verify_out]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import supervision as sv

from digital_shadow.config import (
    load_detector_config,
    load_site_config,
    load_tracker_config,
)
from digital_shadow.perception import YoloDetector
from digital_shadow.sources.file_source import OpenCVFrameReader
from digital_shadow.tracking import (
    reconnector_from_config,
    tracked_to_sv,
    tracker_from_config,
)

CLASS_NAMES = {2: "car", 5: "bus", 7: "truck"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--max-seconds", type=float, default=None)
    p.add_argument("--out-dir", default="verify_out")
    p.add_argument("--sample-every", type=int, default=500)
    p.add_argument("--dump-frames", default="",
                   help="índices de frame extra a volcar anotados (coma-separados), "
                        "p. ej. para inspeccionar eventos de oclusión concretos")
    p.add_argument("--tracker", default=None, choices=["bytetrack", "ocsort"],
                   help="fuerza el tracker (ablación); por defecto, el de config")
    p.add_argument("--out-name", default="annotated_tracks",
                   help="nombre base del mp4/PNGs (para no pisar entre trackers)")
    p.add_argument("--reid", default=None, choices=["on", "off"],
                   help="fuerza la capa de re-id on/off; por defecto, config")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()
    trk_cfg = load_tracker_config()
    if args.reid is not None:
        from dataclasses import replace
        trk_cfg = replace(trk_cfg, reid=replace(trk_cfg.reid, enabled=(args.reid == "on")))
    fps = float(site.fps_ref)

    active_tracker = (args.tracker or trk_cfg.tracker_type).lower()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    video_out = out_dir / f"{args.out_name}.mp4"

    print("=" * 70)
    print(f"VALIDACIÓN VISUAL DEL TRACKING ({active_tracker}) — IDs + estela")
    print(f"  conf {det_cfg.confidence} | lost_track_buffer {site.lost_track_buffer_frames} frames")
    print(f"  salida: {video_out}")
    print("=" * 70)

    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    detector.warmup(site.frame_height, site.frame_width)
    tracker = tracker_from_config(trk_cfg, site, override_type=args.tracker)
    reconnector = reconnector_from_config(trk_cfg, site)

    # Stable color per track_id; trajectory trail of ~2 s.
    box_ann = sv.BoxAnnotator(color_lookup=sv.ColorLookup.TRACK, thickness=2)
    label_ann = sv.LabelAnnotator(color_lookup=sv.ColorLookup.TRACK, text_scale=0.5)
    trace_ann = sv.TraceAnnotator(
        color_lookup=sv.ColorLookup.TRACK, trace_length=int(2 * fps)
    )

    reader = OpenCVFrameReader(site.video_path)
    writer = cv2.VideoWriter(
        str(video_out), cv2.VideoWriter_fourcc(*"mp4v"), fps, site.resolution
    )
    extra_dump = {int(x) for x in args.dump_frames.split(",") if x.strip()}
    index = 0
    written = 0
    while reader.grab():
        if args.max_seconds is not None and index / fps > args.max_seconds:
            break
        image = reader.retrieve()
        tracked = tracker.update(detector.detect(image))
        tracked = reconnector.apply(tracked, image, index)
        dets = tracked_to_sv(tracked)

        annotated = image
        # The annotators require the tracker_id field; on frames without confirmed
        # tracks (empty Detections) the frame is written as is.
        if len(dets):
            labels = [
                f"#{tid} {CLASS_NAMES.get(int(cid), cid)}"
                for tid, cid in zip(dets.tracker_id, dets.class_id)
            ]
            annotated = trace_ann.annotate(annotated, dets)
            annotated = box_ann.annotate(annotated, dets)
            annotated = label_ann.annotate(annotated, dets, labels=labels)

        writer.write(annotated)
        if index % args.sample_every == 0 or index in extra_dump:
            cv2.imwrite(str(out_dir / f"{args.out_name}_{index:05d}.png"), annotated)
        written += 1
        index += 1

    writer.release()
    reader.release()

    print("\nRESULTADO")
    print("-" * 70)
    print(f"  frames anotados : {written}")
    print(f"  video           : {video_out.resolve()}")
    print(f"  PNGs muestreados: cada {args.sample_every} en {out_dir.resolve()}")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

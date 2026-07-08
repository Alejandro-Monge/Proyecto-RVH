"""Visual validation — annotates the detections over the video (step deliverable).

Pass WITHOUT pacing (full coverage, so the video is smooth and each vehicle can be
judged, not the real-time subset): draws boxes and `class conf` on
each frame and encodes to mp4 at `fps_ref`, plus a few sampled PNGs. It does NOT yet
compare against the ground truth (that comes after tracking); it is so a human can
judge: does it detect well? reasonable classes? do far or occluded ones slip through?

Usage:
  python -m scripts.annotate_detections [--max-seconds S] [--conf C]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from digital_shadow.config import load_detector_config, load_site_config
from digital_shadow.perception import YoloDetector
from digital_shadow.perception.types import Detection
from digital_shadow.sources.file_source import OpenCVFrameReader

# BGR colors per class (stable for judging by eye).
CLASS_COLORS = {
    "car": (0, 200, 0),
    "bus": (0, 140, 255),
    "truck": (255, 80, 0),
    "motorcycle": (180, 0, 200),
}
_DEFAULT_COLOR = (200, 200, 200)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--max-seconds", type=float, default=None,
                   help="acota la ventana de medio (s); sin él, tramo completo")
    p.add_argument("--conf", type=float, default=None,
                   help="sobreescribe el umbral de confianza (para ver low-conf)")
    p.add_argument("--out-dir", default="verify_out")
    p.add_argument("--sample-every", type=int, default=500,
                   help="guarda un PNG cada N frames anotados")
    return p.parse_args()


def _draw(image, detections: list[Detection]) -> None:
    for d in detections:
        color = CLASS_COLORS.get(d.class_name, _DEFAULT_COLOR)
        p1 = (int(d.x1), int(d.y1))
        p2 = (int(d.x2), int(d.y2))
        cv2.rectangle(image, p1, p2, color, 2)
        label = f"{d.class_name} {d.confidence:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(image, (p1[0], p1[1] - th - 4), (p1[0] + tw, p1[1]), color, -1)
        cv2.putText(image, label, (p1[0], p1[1] - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()
    if args.conf is not None:
        from dataclasses import replace
        det_cfg = replace(det_cfg, confidence=args.conf)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    video_out = out_dir / "annotated.mp4"

    print("=" * 70)
    print("VALIDACIÓN VISUAL — anotación de detecciones (sin pacing, cobertura total)")
    print(f"  modelo {det_cfg.weights} half={det_cfg.half} conf={det_cfg.confidence} "
          f"clases={list(site.class_set)}")
    print(f"  salida: {video_out}")
    print("=" * 70)

    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    detector.warmup(site.frame_height, site.frame_width)

    reader = OpenCVFrameReader(site.video_path)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(
        str(video_out), fourcc, float(site.fps_ref), site.resolution
    )

    fps = float(site.fps_ref)
    index = 0
    written = 0
    n_detections = 0
    while reader.grab():
        ts = index / fps
        if args.max_seconds is not None and ts > args.max_seconds:
            break
        image = reader.retrieve()
        detections = detector.detect(image)
        n_detections += len(detections)
        _draw(image, detections)
        writer.write(image)
        if index % args.sample_every == 0:
            cv2.imwrite(str(out_dir / f"annotated_{index:05d}.png"), image)
        written += 1
        index += 1

    writer.release()
    reader.release()

    print("\nRESULTADO")
    print("-" * 70)
    print(f"  frames anotados            : {written}")
    print(f"  detecciones totales        : {n_detections}")
    print(f"  media detecciones/frame    : {n_detections / written:.2f}" if written else "  (sin frames)")
    print(f"  video                      : {video_out.resolve()}")
    print(f"  PNGs muestreados cada {args.sample_every}: {out_dir.resolve()}")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

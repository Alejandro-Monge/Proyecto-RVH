"""Verification of the video source interface over the real segment.

Traverses `FileVideoSource` in real-time mode (REAL clock) over the site video
and reports: delivered frames, dropped, drop rate, timestamp of the first
and last frame, `end_of_source` detection, and THE INVARIANT
`delivered + dropped == total` (the first number to look at: if it does not add up, the
drop rate is a lie).

Simulated slow consumer (no real detector): `--work-ms` introduces a per-frame
delay that acts as the 'processing cost' of the nonexistent detector, to
trigger dropping at will.

Honest note: a real-time pass of the 180 s segment takes ~180 s of wall-clock by
construction (the source is anchored to real time). `--max-seconds` bounds the
window to iterate quickly; without it, the full segment is traversed until EOF.

Examples:
  # Slow consumer (10 fps), 20 s window -> high dropping
  python -m scripts.verify_video_source --work-ms 100 --max-seconds 20
  # Fast consumer, 20 s window -> dropping ~0, the source waits
  python -m scripts.verify_video_source --work-ms 0 --max-seconds 20
  # Full pass until EOF (~180 s) -> confirms EOF and the invariant == 5325
  python -m scripts.verify_video_source
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

from digital_shadow.config import load_site_config
from digital_shadow.sources import file_source_from_site


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia", help="id del sitio (config/sites/<id>)")
    p.add_argument(
        "--work-ms",
        type=float,
        default=0.0,
        help="retardo por frame que simula el coste del consumidor (ms)",
    )
    p.add_argument(
        "--max-seconds",
        type=float,
        default=None,
        help="acota la ventana de medio a procesar (s); sin él, hasta EOF",
    )
    p.add_argument(
        "--dump-frames",
        action="store_true",
        help="guarda el primer y el último frame entregado como PNG",
    )
    p.add_argument(
        "--dump-dir",
        default="verify_out",
        help="directorio donde guardar los frames con --dump-frames",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    work_s = args.work_ms / 1000.0

    print("=" * 70)
    print(f"Verificación de fuente de video — sitio: {site.site_id}")
    print(f"  video        : {site.video_path}")
    print(f"  fps_ref      : {site.fps_ref} (= {float(site.fps_ref):.4f} fps)")
    print(f"  resolución   : {site.frame_width}x{site.frame_height}")
    print(f"  total tramo  : {site.expected_total_frames} frames "
          f"(= ventana [{site.t0_s}, {site.t1_s}] s a CFR)")
    print(f"  consumidor   : work={args.work_ms:.1f} ms/frame "
          f"(simula coste del detector inexistente)")
    print(f"  ventana max  : {'tramo completo' if args.max_seconds is None else f'{args.max_seconds} s'}")
    print("=" * 70)

    first_ts = None
    last_ts = None
    first_image = None
    last_image = None
    reached_eof = False
    res_ok = True

    wall_start = time.monotonic()
    with file_source_from_site(site) as source:
        while True:
            frame = source.read()
            if frame.end_of_source:
                reached_eof = True
                break

            if first_ts is None:
                first_ts = frame.timestamp_s
                first_image = frame.image
            last_ts = frame.timestamp_s
            last_image = frame.image

            # Sanity: each delivered frame is a real image with the config
            # resolution (height, width) -> shape (h, w, channels).
            h, w = frame.image.shape[:2]
            if (w, h) != site.resolution:
                res_ok = False

            # Window cut by measurement time (it does not trim the source; the
            # consumer decides to stop asking).
            if args.max_seconds is not None and frame.timestamp_s >= args.max_seconds:
                break

            if work_s > 0:
                time.sleep(work_s)

        delivered = source.delivered_frames
        discarded = source.discarded_frames

    wall_elapsed = time.monotonic() - wall_start
    total_consumed = delivered + discarded
    discard_rate = 100.0 * discarded / total_consumed if total_consumed else 0.0

    print("\nRESULTADO")
    print("-" * 70)
    # The first number: the invariant.
    print(f"  entregados + descartados   : {delivered} + {discarded} = {total_consumed}")
    if reached_eof:
        match = (total_consumed == site.expected_total_frames)
        print(f"  INVARIANTE vs total tramo  : {total_consumed} == {site.expected_total_frames}  "
              f"-> {'OK' if match else 'FALLO'}")
    else:
        print(f"  (ventana acotada; no se alcanzó EOF, total < {site.expected_total_frames})")
    print(f"  tasa de descarte           : {discard_rate:.2f} %  (descartados / consumidos)")
    print(f"  timestamp primer frame     : {first_ts}")
    print(f"  timestamp último frame     : {last_ts}")
    print(f"  end_of_source detectado    : {reached_eof}")
    print(f"  resolución de frames OK    : {res_ok}")
    print(f"  tiempo de reloj de la corrida: {wall_elapsed:.2f} s")
    print("-" * 70)

    if args.dump_frames and first_image is not None:
        out = Path(args.dump_dir)
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / "first_frame.png"), first_image)
        cv2.imwrite(str(out / "last_frame.png"), last_image)
        print(f"  frames guardados en        : {out.resolve()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

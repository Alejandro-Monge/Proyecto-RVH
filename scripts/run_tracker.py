"""Canonical tracking run (real-time): CSV trajectories + dropping + sanity.

Runs the OPERATIONAL pipeline `source(real-time) -> detector -> tracker` until EOF,
persists the raw trajectories to CSV (input of the later stage) and reports:
  * DROP rate (re-verified: adding ByteTrack must not break real-time);
  * SANITY CHECK of the number of tracks vs the 72 of the GT — NOT a fidelity metric.

The detector is warmed up BEFORE opening the source (the CUDA init must not count
as dropping). The tracker is a single instance, update() per frame in order.

Usage:
  python -m scripts.run_tracker [--out verify_out/tracks_rusia.csv] [--max-seconds S]
"""

from __future__ import annotations

import argparse
import csv
import time
from collections import Counter
from pathlib import Path

from digital_shadow.config import (
    load_detector_config,
    load_site_config,
    load_tracker_config,
)
from digital_shadow.perception import YoloDetector
from digital_shadow.sources import file_source_from_site
from digital_shadow.tracking import reconnector_from_config, tracker_from_config


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--site", default="rusia")
    p.add_argument("--out", default="verify_out/tracks_rusia.csv")
    p.add_argument("--max-seconds", type=float, default=None)
    p.add_argument("--min-track-frames", type=int, default=5,
                   help="umbral para contar un track como 'persistente' en el sanity")
    p.add_argument("--tracker", default=None, choices=["bytetrack", "ocsort"],
                   help="fuerza el tracker (ablación); por defecto, el de config")
    p.add_argument("--reid", default=None, choices=["on", "off"],
                   help="fuerza la capa de re-id on/off (ablación); por defecto, config")
    p.add_argument("--no-pacing", action="store_true",
                   help="procesa TODOS los frames sin descarte (ablación de beneficio "
                        "limpia); el coste/descarte de la re-id se mide en real-time aparte")
    return p.parse_args()


CSV_HEADER = [
    "timestamp_s", "frame_index", "track_id",
    "x1", "y1", "x2", "y2", "cx", "cy", "class_id", "confidence",
]


def main() -> int:
    args = parse_args()
    site = load_site_config(args.site)
    det_cfg = load_detector_config()
    trk_cfg = load_tracker_config()
    if args.reid is not None:  # switch override for the ablation
        from dataclasses import replace
        trk_cfg = replace(trk_cfg, reid=replace(trk_cfg.reid, enabled=(args.reid == "on")))
    fps = float(site.fps_ref)

    active_tracker = (args.tracker or trk_cfg.tracker_type).lower()
    print("=" * 70)
    print(f"TRACKING — corrida canónica real-time (detector -> {active_tracker})")
    print(f"  conf detector {det_cfg.confidence} | lost_track_buffer "
          f"{site.lost_track_buffer_frames} frames (~{site.lost_track_buffer_frames/fps:.1f} s)")
    print(f"  tracker.yaml: act={trk_cfg.track_activation_threshold} "
          f"high={trk_cfg.high_conf_det_threshold} iou={trk_cfg.minimum_iou_threshold} "
          f"min_consec={trk_cfg.minimum_consecutive_frames}")
    print(f"  salida CSV: {args.out}")
    print("=" * 70)

    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    detector.warmup(site.frame_height, site.frame_width)
    tracker = tracker_from_config(trk_cfg, site, override_type=args.tracker)
    reconnector = reconnector_from_config(trk_cfg, site)
    print(f"  re-id: {'ON (' + trk_cfg.reid.model + ')' if trk_cfg.reid.enabled else 'OFF (passthrough)'}")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    track_frame_counts: Counter = Counter()
    n_rows = 0
    wall_start = time.monotonic()
    with open(out_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(CSV_HEADER)

        def process(image, ts, fidx):
            nonlocal n_rows
            tracked = tracker.update(detector.detect(image))
            tracked = reconnector.apply(tracked, image, fidx)
            for t in tracked:
                writer.writerow([
                    f"{ts:.6f}", fidx, t.track_id,
                    f"{t.x1:.2f}", f"{t.y1:.2f}", f"{t.x2:.2f}", f"{t.y2:.2f}",
                    f"{t.cx:.2f}", f"{t.cy:.2f}", t.class_id, f"{t.confidence:.4f}",
                ])
                track_frame_counts[t.track_id] += 1
                n_rows += 1

        if args.no_pacing:
            # Processes ALL frames (no dropping) for a clean BENEFIT ablation:
            # the re-id cost is measured separately in real-time mode.
            from digital_shadow.sources.file_source import OpenCVFrameReader
            reader = OpenCVFrameReader(site.video_path)
            idx = 0
            while reader.grab():
                ts = idx / fps
                if args.max_seconds is not None and ts > args.max_seconds:
                    break
                process(reader.retrieve(), ts, idx)
                idx += 1
            reader.release()
            delivered, discarded = idx, 0
        else:
            with file_source_from_site(site) as source:
                while True:
                    frame = source.read()
                    if frame.end_of_source:
                        break
                    ts = frame.timestamp_s
                    fidx = round(ts * fps)
                    process(frame.image, ts, fidx)
                    if args.max_seconds is not None and ts >= args.max_seconds:
                        break
                delivered = source.delivered_frames
                discarded = source.discarded_frames

    wall = time.monotonic() - wall_start
    consumed = delivered + discarded
    discard_rate = 100.0 * discarded / consumed if consumed else 0.0
    n_tracks = len(track_frame_counts)
    persistent = sum(1 for c in track_frame_counts.values() if c >= args.min_track_frames)

    print("\nRESULTADO")
    print("-" * 70)
    print(f"  frames procesados / descartados : {delivered} / {discarded}")
    print(f"  TASA DE DESCARTE                : {discard_rate:.2f} %  "
          f"(re-verifica que el tracking no rompe real-time)")
    print(f"  observaciones de track (filas)  : {n_rows}")
    print(f"  CSV de trayectorias             : {out_path.resolve()}")
    print(f"  tiempo de reloj                 : {wall:.1f} s")
    print("\n  SANITY CHECK (NO es métrica de fidelidad de ruta):")
    print(f"    tracks únicos confirmados       : {n_tracks}")
    print(f"    tracks persistentes (>= {args.min_track_frames} frames): {persistent}")
    print(f"    GT (referencia de orden de magnitud): 72 vehículos `complete`")
    print("    CAVEAT: el conteo está INFLADO por (a) el parqueadero (vehículos")
    print("    parados, deuda de Fase 3 aún no excluida) y (b) fragmentación de IDs.")
    print("    El número FINO no es interpretable hasta Fase 3 (zonas + métrica de")
    print("    ruta). Esto es cordura gruesa de orden de magnitud, no fidelidad.")
    print("-" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

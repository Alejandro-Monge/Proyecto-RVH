"""Section 7.6 of the FULL LIVE loop (Part 5): REAL detector + tracking running over the video,
+ orchestration + SUMO drawing, time per stage measured (not cited). Does it hold wall-clock (33.8 ms)?

Measures the detector (YOLOv8x) and the tracker (ByteTrack) processing real video frames (GPU), per
stage; adds the orchestration and SUMO drawing already measured (sub-ms). Over one segment (not the whole clip).
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from digital_shadow.config import load_detector_config, load_tracker_config
from digital_shadow.config.site import load_site_config
from digital_shadow.perception import YoloDetector
from digital_shadow.tracking import tracker_from_config

DT = 12 / 355
N = 600       # frames of the segment to measure
WARM = 20     # warmup frames (discarded from the mean)


def main():
    site = load_site_config("rusia")
    det = YoloDetector(load_detector_config(), list(site.class_set)).load()
    trk = tracker_from_config(load_tracker_config(), site)
    cap = cv2.VideoCapture(str(ROOT / "data/sources/rusia/rusia_tramo_000-180.mp4"))
    cap.set(cv2.CAP_PROP_POS_FRAMES, 1200)   # segment with traffic

    t_det, t_trk = [], []
    for i in range(N):
        ok, frame = cap.read()
        if not ok:
            break
        a = time.perf_counter(); dets = det.detect(frame); b = time.perf_counter()
        trk.update(dets); c = time.perf_counter()
        if i >= WARM:
            t_det.append((b - a) * 1000); t_trk.append((c - b) * 1000)
    cap.release()

    det_ms = float(np.median(t_det)); det_p90 = float(np.percentile(t_det, 90))
    trk_ms = float(np.median(t_trk)); trk_p90 = float(np.percentile(t_trk, 90))
    orch_ms, sumo_ms = 0.51, 0.07   # measured in Part 4 (sub-ms)
    budget = 1000 * DT

    print(f"=== §7.6 BUCLE COMPLETO EN VIVO ({len(t_det)} frames medidos, GPU real) ===")
    print(f"  {'etapa':28s} {'mediana':>8s} {'p90':>8s}  (ms/frame)")
    print(f"  {'detector (YOLOv8x FP16)':28s} {det_ms:8.2f} {det_p90:8.2f}  MEDIDO EN VIVO aquí")
    print(f"  {'tracking (ByteTrack)':28s} {trk_ms:8.2f} {trk_p90:8.2f}  MEDIDO EN VIVO aquí")
    print(f"  {'orquestación causal':28s} {orch_ms:8.2f} {'-':>8s}  medido Parte 4")
    print(f"  {'dibujo SUMO (sync+move)':28s} {sumo_ms:8.2f} {'-':>8s}  medido Parte 4")
    total = det_ms + trk_ms + orch_ms + sumo_ms
    total_p90 = det_p90 + trk_p90 + orch_ms + sumo_ms
    print(f"  {'TOTAL bucle':28s} {total:8.2f} {total_p90:8.2f}  vs presupuesto {budget:.1f} ms/frame")
    print(f"  -> {'MANTIENE wall-clock' if total < budget else 'NO mantiene → delay de cola'} "
          f"(mediana ×{budget/total:.1f}; pico p90 ×{budget/total_p90:.1f}); el detector domina.")
    print(f"  Honestidad: si en congestión real el pico superara {budget:.0f} ms, sería delay de COLA a")
    print(f"  declarar (el sistema se rezagaría del wall-clock). Aquí el margen mediana/p90 lo evita.")


if __name__ == "__main__":
    main()

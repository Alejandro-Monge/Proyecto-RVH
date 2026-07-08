"""Tracking fragmentation BASELINE (MEASUREMENT only — does not reconnect, does not touch the tracker).

Measures how many tracks the baseline produces today (verify_out/tracks_rusia.csv = ByteTrack-140,
reid OFF) against the 73 real vehicles of GT v2, and CHARACTERIZES WHERE it fragments.
It does not assume the cause: it reports what the data show (region, field, gap, class).

Filters (transparent and reported):
  - Parking lot: tracks whose GROUND CONTACT (gx,gy)=((x1+x2)/2, y2) falls inside the
    WEST exclusion polygon (config/sites/rusia/zones.yaml) are discarded, by membership of
    the MAJORITY of their observations. The SOUTH cluster (red queue = real traffic) is NOT excluded.
  - Noise: the count is reported at several minimum-track-length thresholds (sensitivity),
    not fixed by eye.

Fragmentation events (death of track A -> birth of B nearby in time/space),
anchored on ground contact, with the same parameters as scripts/catalog_fragmentation_events.py
(for comparability). They are binned by image region (corner->approach, coarse PROXY, NOT the
calibrated zone), near/far field, gap length and class.

Usage: python -m scripts.measure_fragmentation_baseline
"""
from __future__ import annotations

import csv
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
CSV = ROOT / "verify_out" / "tracks_rusia.csv"
ZONES = ROOT / "config" / "sites" / "rusia" / "zones.yaml"
BG = ROOT / "verify_out" / "homography_improved" / "clean_frame_2541.png"
OUT_DIR = ROOT / "verify_out" / "fragmentation_baseline"
GT_N = 73
FPS = 355 / 12
W, H = 1200, 646
CLS = {2: "car", 5: "bus", 7: "truck"}

# Fragmentation event parameters (identical to catalog_fragmentation_events.py)
MIN_GAP, MAX_GAP, MAX_DIST = 20, 150, 250.0


def quadrant_arm(x, y):
    """Corner->approach (coarse image PROXY, NOT the calibrated zone). Corner mapping
    (docs/site_rusia_geometry.md Section 2): top-left=SUR, top-right=OESTE, bottom-left=ESTE, bottom-right=NORTE."""
    top = y < H / 2
    left = x < W / 2
    if top and left:
        return "SUR (sup-izq, lejano)"
    if top and not left:
        return "OESTE (sup-der, lejano)"
    if not top and left:
        return "ESTE (inf-izq, medio)"
    return "NORTE (inf-der, cercano)"


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    poly = np.array(yaml.safe_load(ZONES.read_text())["exclusion_zones"]
                    ["oeste_parqueadero"]["polygon_px"], dtype=np.float32)

    # track_id -> list of obs (frame, x1,y1,x2,y2, cls)
    obs = defaultdict(list)
    with CSV.open() as fh:
        for r in csv.DictReader(fh):
            obs[int(r["track_id"])].append((
                int(r["frame_index"]), float(r["x1"]), float(r["y1"]),
                float(r["x2"]), float(r["y2"]), int(r["class_id"])))
    for v in obs.values():
        v.sort()

    def gc(o):  # ground contact (gx, gy)
        return ((o[1] + o[3]) / 2.0, o[4])

    # --- Parking classification (by majority of ground contact inside the polygon) ---
    parking, frac_in = set(), {}
    for tid, v in obs.items():
        inside = sum(1 for o in v if cv2.pointPolygonTest(poly, gc(o), False) >= 0)
        frac_in[tid] = inside / len(v)
        if frac_in[tid] >= 0.5:
            parking.add(tid)

    non_parking = {t: v for t, v in obs.items() if t not in parking}

    print("=" * 72)
    print("LÍNEA BASE DE FRAGMENTACIÓN — tracks_rusia.csv (ByteTrack-140, reid OFF)")
    print("=" * 72)
    print(f"GT v2 = {GT_N} vehículos reales\n")

    # --- Counting funnel ---
    print("EMBUDO DE CONTEO")
    print(f"  tracks únicos totales (confirmados, id>=0) : {len(obs)}")
    print(f"  excluidos por parqueadero (zona OESTE)     : {len(parking)}  "
          f"-> {sorted(parking)}")
    for tid in sorted(parking):
        print(f"        tid {tid}: {len(obs[tid])} obs, {frac_in[tid]*100:.0f}% dentro del polígono")
    print(f"  tracks NO-parqueadero                      : {len(non_parking)}")
    print("\n  Sensibilidad al umbral de ruido (longitud mínima de track, en frames):")
    print(f"    {'umbral':>8} {'tracks':>8} {'vs 73':>8}  {'inflación':>10}")
    for thr in (1, 3, 5, 10, 15, 30):
        n = sum(1 for v in non_parking.values() if len(v) >= thr)
        print(f"    {thr:>6}f {n:>8} {n-GT_N:>+8} {n/GT_N:>9.2f}x")
    PRIMARY = 5
    counted = {t: v for t, v in non_parking.items() if len(v) >= PRIMARY}
    print(f"\n  >>> CONTEO BASE (no-parqueadero, >= {PRIMARY} frames) = {len(counted)} tracks "
          f"vs {GT_N} vehículos  =>  {len(counted)/GT_N:.2f}x  "
          f"({len(counted)-GT_N} de exceso por fragmentar)")

    # --- Fragmentation events (death A -> birth B), anchored on ground contact ---
    births = {t: (v[0][0], *gc(v[0]), v[0][5], v[0][4] - v[0][2]) for t, v in non_parking.items()}
    deaths = {t: (v[-1][0], *gc(v[-1]), v[-1][5], v[-1][4] - v[-1][2]) for t, v in non_parking.items()}
    frags = []
    for a, (fa, xa, ya, ca, ha) in deaths.items():
        best = None
        for b, (fb, xb, yb, cb, hb) in births.items():
            if b == a:
                continue
            gap = fb - fa
            if not (MIN_GAP <= gap <= MAX_GAP):
                continue
            d = float(np.hypot(xb - xa, yb - ya))
            if d > MAX_DIST:
                continue
            if best is None or gap < best[0]:
                best = (gap, b, fb, xb, yb, d, ca, ha)
        if best is not None:
            gap, b, fb, xb, yb, d, ca, ha = best
            mx, my = (xa + xb) / 2, (ya + yb) / 2
            frags.append(dict(a=a, b=b, gap=gap, dist=d, mx=mx, my=my, cls=ca,
                              h=ha, gap_s=gap / FPS))

    print("\n" + "=" * 72)
    print(f"EVENTOS DE FRAGMENTACIÓN (muerte->nacimiento; gap {MIN_GAP}-{MAX_GAP}f, dist<={MAX_DIST:.0f}px)")
    print(f"  total de eventos de fragmentación: {len(frags)}")

    def dist_table(title, keyfn, order=None):
        c = Counter(keyfn(e) for e in frags)
        print(f"\n  {title}")
        keys = order or sorted(c, key=lambda k: -c[k])
        for k in keys:
            n = c.get(k, 0)
            bar = "█" * n
            print(f"    {str(k):<26} {n:>3}  {bar}")

    dist_table("por REGIÓN de imagen (esquina->brazo; PROXY grueso, no zona Fase 3):",
               lambda e: quadrant_arm(e["mx"], e["my"]),
               ["SUR (sup-izq, lejano)", "OESTE (sup-der, lejano)",
                "ESTE (inf-izq, medio)", "NORTE (inf-der, cercano)"])
    dist_table("por CAMPO (mitad superior=lejano/horizonte, inferior=cercano):",
               lambda e: "LEJANO (mitad superior)" if e["my"] < H / 2 else "CERCANO (mitad inferior)",
               ["LEJANO (mitad superior)", "CERCANO (mitad inferior)"])
    dist_table("por HUECO de oclusión (duración muerte->nacimiento):",
               lambda e: ("corto 0.7-1.5s" if e["gap_s"] < 1.5 else
                          "medio 1.5-3s" if e["gap_s"] < 3 else "largo >=3s (regimen bus)"),
               ["corto 0.7-1.5s", "medio 1.5-3s", "largo >=3s (regimen bus)"])
    dist_table("por CLASE en la muerte del fragmento:",
               lambda e: CLS.get(e["cls"], e["cls"]))

    # the combination that matters most: long gaps + class, with their location
    print("\n  Los 10 eventos de hueco MÁS LARGO (candidatos a oclusión por bus):")
    print(f"    {'A->B':>9} {'gap_s':>6} {'dist':>5} {'clase':>6}  región")
    for e in sorted(frags, key=lambda e: -e["gap"])[:10]:
        print(f"    {str(e['a'])+'->'+str(e['b']):>9} {e['gap_s']:>6.2f} {e['dist']:>5.0f} "
              f"{CLS.get(e['cls'], e['cls']):>6}  {quadrant_arm(e['mx'], e['my'])}")

    # --- Distribution of the counted tracks by region (where the excess is) ---
    print("\n" + "=" * 72)
    print("DÓNDE ESTÁN LOS TRACKS CONTADOS (por región de su contacto-suelo MEDIANO):")
    by_reg = Counter()
    for v in counted.values():
        gxs = [gc(o)[0] for o in v]; gys = [gc(o)[1] for o in v]
        by_reg[quadrant_arm(np.median(gxs), np.median(gys))] += 1
    for k in ["SUR (sup-izq, lejano)", "OESTE (sup-der, lejano)",
              "ESTE (inf-izq, medio)", "NORTE (inf-der, cercano)"]:
        print(f"    {k:<26} {by_reg.get(k,0):>3} tracks")

    # --- Scatter PNG of the break points over a frame ---
    if BG.exists():
        img = cv2.imread(str(BG))
        cv2.polylines(img, [poly.astype(np.int32)], True, (0, 165, 255), 2)
        cv2.putText(img, "exclusion parqueadero", (int(poly[:,0].min()), int(poly[:,1].min())-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        for e in frags:
            col = (0, 0, 255) if e["gap_s"] >= 3 else (0, 200, 255) if e["gap_s"] >= 1.5 else (0, 255, 0)
            cv2.circle(img, (int(e["mx"]), int(e["my"])), 7, col, 2)
        cv2.putText(img, "rupturas: verde<1.5s  ambar 1.5-3s  rojo>=3s", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out_png = OUT_DIR / "fragmentation_breaks.png"
        cv2.imwrite(str(out_png), img)
        print(f"\n  scatter de rupturas: {out_png}")

    # --- Events CSV for the video render / inspection ---
    out_csv = OUT_DIR / "fragmentation_events_baseline.csv"
    with out_csv.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["track_a", "track_b", "gap_frames", "gap_s", "dist_px",
                    "mid_x", "mid_y", "class", "region"])
        for e in sorted(frags, key=lambda e: -e["gap"]):
            w.writerow([e["a"], e["b"], e["gap"], f"{e['gap_s']:.2f}", f"{e['dist']:.0f}",
                        f"{e['mx']:.0f}", f"{e['my']:.0f}", CLS.get(e["cls"], e["cls"]),
                        quadrant_arm(e["mx"], e["my"]).split(" ")[0]])
    print(f"  catálogo de eventos: {out_csv}")
    print("=" * 72)


if __name__ == "__main__":
    main()

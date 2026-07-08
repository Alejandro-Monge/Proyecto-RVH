"""Validates the CLASS STABILIZATION (simple majority vote) against GT v2 (vehicle_class).
Measures accuracy and STABILITY, before (per-frame, no vote) vs after (vote), per class. On
RECONNECTED tracks, subset matched with GT. It does NOT write artifacts.

Primary vote = SIMPLE (per-frame frequency; MOT standard). Weighted variants (area,
confidence) are reported as EXPLORED with no conclusive improvement on this sample (small N).
Sample caveat: heavily imbalanced classes (car 58 / bus 11 / truck 4) → it is reported PER-CLASS
(the aggregate is inflated by the cars: 'always car' ≈ 78%); truck N=4 is anecdotal (huge CI).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte  # CLASS_BY_COCO

CLASSES = ["car", "bus", "truck"]


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d
    return max(0.0, c - h), min(1.0, c + h)


def main():
    coco = dte.CLASS_BY_COCO
    ev = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/track_events.csv")
    voted = {t: (v if isinstance(v, str) else "") for t, v in zip(ev.track_id, ev.vehicle_class.fillna(""))}
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv").set_index("event_id")
    matches = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    tracks = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/tracks_reconnected.csv")
    tracks["cls"] = tracks.class_id.map(coco)
    tracks["area"] = (tracks.x2 - tracks.x1) * (tracks.y2 - tracks.y1)
    by = {t: g for t, g in tracks.groupby("track_id")}

    rec = []
    for _, m in matches.iterrows():
        g = by.get(int(m.track_id)); gc = gt.loc[m.event_id].vehicle_class
        if g is None:
            continue
        simple = g.cls.value_counts().idxmax()
        conf = g.groupby("cls").confidence.sum().idxmax()
        area = g.groupby("cls").area.sum().idxmax()
        rec.append(dict(tid=int(m.track_id), klass=gc, deployed=voted.get(int(m.track_id), ""),
                        simple=simple, conf=conf, area=area,
                        pf=(g.cls == gc).mean(), nfr=len(g), ncls=g.cls.nunique()))
    d = pd.DataFrame(rec)

    print("=" * 84)
    print("VALIDACIÓN de estabilización de clase (voto simple) vs GT v2 — pares casados")
    print("=" * 84)
    print(f"N casados {len(d)} | GT: " + ", ".join(f"{c} {int((d.klass==c).sum())}" for c in CLASSES))

    # BEFORE vs AFTER
    pf = d.pf.mean()  # pooled per-frame accuracy (expectation of trusting a single detection)
    blink = (d.ncls > 1).mean()
    acc = (d.deployed == d.klass).mean()
    print(f"\nANTES (sin voto): acierto per-frame {pf*100:.0f}% | estabilidad: {blink*100:.0f}% de tracks BLINKEAN")
    print(f"DESPUÉS (voto): acierto {acc*100:.0f}% | estabilidad 100% por construcción (1 clase/track)")
    print(f"Floor 'siempre car': {(d.klass=='car').mean()*100:.0f}%")

    # PER-CLASS (recall) + CI — what matters (the aggregate is inflated by the cars)
    print("\nRECALL POR CLASE (voto desplegado) con IC95 Wilson — N pequeño en bus/truck:")
    for c in CLASSES:
        s = d[d.klass == c]; ok = int((s.deployed == c).sum()); n = len(s)
        lo, hi = wilson(ok, n)
        flag = "  (anecdótico, N=4)" if c == "truck" else ""
        print(f"  {c:6}: {ok}/{n} = {ok/n*100 if n else 0:3.0f}%  IC95[{lo*100:.0f},{hi*100:.0f}]%{flag}")

    print("\nMATRIZ DE CONFUSIÓN (filas GT, cols voto):")
    print(pd.crosstab(d.klass, d.deployed).reindex(index=CLASSES, columns=CLASSES, fill_value=0))

    # EXPLORED variants (honesty about N)
    print("\nVARIANTES de voto exploradas (acierto per-clase) — diferencias dentro del ruido (N bus/truck):")
    for col in ["simple", "conf", "area"]:
        line = f"  {col:6}: global {(d[col]==d.klass).mean()*100:3.0f}%"
        for c in CLASSES:
            s = d[d.klass == c]; line += f" | {c} {int((s[col]==c).sum())}/{len(s)}"
        print(line)
    print("  → el voto simple (estándar/citable) es el primario; área/confianza no mejoran de forma")
    print("    concluyente con esta muestra (1-2 tracks sobre N=11 bus / N=4 truck). Trabajo futuro si N crece.")


if __name__ == "__main__":
    main()

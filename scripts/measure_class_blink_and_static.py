"""Do class blinking and parked vehicles explain the fragmentation? (MEASUREMENT ONLY.)

Three measurements over the baseline (verify_out/tracks_rusia.csv) and the final cross-check. It does not reconnect,
does not touch the tracker or zones.

Mechanical context (verified in the `trackers` source): ByteTrack associates ONLY by IoU
(class-agnostic); the tracklet does update(bbox), does not use the class; and the detector filters to
{car,bus,truck}, so a car<->truck blink does NOT delete the box. => a class change CANNOT
split an ID by itself. If a break coincides with a class change, it is CORRELATION (the same
hard frame), not cause; stabilizing the class would NOT reduce the fragmentation (it does fix the LABEL).

M1: of the 122 fragmentation events, how many cross a class change (death A vs birth B)?
M2: how many breaks are of QUASI-STATIC vehicles (stopped at red) vs in real motion?
M3: of the 172 counted tracks, how many change class over their life? (size of the
    class problem, independent of the fragmentation).
Cross-check: of the ~99 excess IDs, how much is class blinking, how much quasi-static, how much
    "real" fragmentation of moving traffic?

Usage: python -m scripts.measure_class_blink_and_static
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
GT_N = 73
FPS = 355 / 12
MIN_GAP, MAX_GAP, MAX_DIST = 20, 150, 250.0
PRIMARY_LEN = 5
STATIC_THR = 30.0   # px: ground-contact span below which the track is "quasi-static"
CLS = {2: "car", 5: "bus", 7: "truck"}


def main():
    poly = np.array(yaml.safe_load(ZONES.read_text())["exclusion_zones"]
                    ["oeste_parqueadero"]["polygon_px"], dtype=np.float32)
    obs = defaultdict(list)
    with CSV.open() as fh:
        for r in csv.DictReader(fh):
            obs[int(r["track_id"])].append((
                int(r["frame_index"]), float(r["x1"]), float(r["y1"]),
                float(r["x2"]), float(r["y2"]), int(r["class_id"])))
    for v in obs.values():
        v.sort()

    def gc(o):
        return ((o[1] + o[3]) / 2.0, o[4])

    def span(v):
        xs = [gc(o)[0] for o in v]; ys = [gc(o)[1] for o in v]
        return float(np.hypot(max(xs) - min(xs), max(ys) - min(ys)))

    def majority_cls(v):
        return Counter(o[5] for o in v).most_common(1)[0][0]

    parking = set()
    for tid, v in obs.items():
        inside = sum(1 for o in v if cv2.pointPolygonTest(poly, gc(o), False) >= 0)
        if inside / len(v) >= 0.5:
            parking.add(tid)
    counted = {t: v for t, v in obs.items() if t not in parking and len(v) >= PRIMARY_LEN}
    non_parking = {t: v for t, v in obs.items() if t not in parking}

    # --- Fragmentation events (same parameters as the baseline) ---
    births = {t: (v[0][0], *gc(v[0]), v[0][5]) for t, v in non_parking.items()}
    deaths = {t: (v[-1][0], *gc(v[-1]), v[-1][5]) for t, v in non_parking.items()}
    frags = []
    for a, (fa, xa, ya, ca) in deaths.items():
        best = None
        for b, (fb, xb, yb, cb) in births.items():
            if b == a:
                continue
            gap = fb - fa
            if not (MIN_GAP <= gap <= MAX_GAP):
                continue
            d = float(np.hypot(xb - xa, yb - ya))
            if d > MAX_DIST:
                continue
            if best is None or gap < best[0]:
                best = (gap, b, d, cb)
        if best is not None:
            gap, b, d, cb = best
            frags.append(dict(a=a, b=b, gap=gap, gap_s=gap / FPS, dist=d,
                              cls_a=ca, cls_b=cb))

    print("=" * 74)
    print("BLINKEO DE CLASE Y CUASI-ESTÁTICOS — desglose de la fragmentación (baseline)")
    print(f"  {len(frags)} eventos de fragmentación | {len(counted)} tracks contados | GT {GT_N}")
    print("=" * 74)

    # ===================== M1: class change at the break =====================
    m1_last = sum(1 for e in frags if e["cls_a"] != e["cls_b"])
    m1_maj = sum(1 for e in frags
                 if majority_cls(obs[e["a"]]) != majority_cls(obs[e["b"]]))
    a_multi = sum(1 for e in frags if len({o[5] for o in obs[e["a"]]}) > 1)
    b_multi = sum(1 for e in frags if len({o[5] for o in obs[e["b"]]}) > 1)
    print("\nM1 — ¿la ruptura cruza un cambio de clase?")
    print(f"  clase(muerte A) != clase(nacimiento B), por ÚLTIMA/PRIMERA obs : {m1_last}/{len(frags)}"
          f"  ({100*m1_last/len(frags):.0f}%)")
    print(f"  clase MAYORITARIA de A != mayoritaria de B                     : {m1_maj}/{len(frags)}"
          f"  ({100*m1_maj/len(frags):.0f}%)")
    print(f"  el track A es multi-clase en su vida                           : {a_multi}/{len(frags)}")
    print(f"  el track B es multi-clase en su vida                           : {b_multi}/{len(frags)}")
    pares = Counter()
    for e in frags:
        if e["cls_a"] != e["cls_b"]:
            pares[tuple(sorted((CLS.get(e["cls_a"]), CLS.get(e["cls_b"]))))] += 1
    print(f"  pares de clase en las rupturas con cambio: {dict(pares)}")
    print("  NOTA: ByteTrack es class-agnóstico (IoU) => esto es CORRELACIÓN, no causa.")

    # ===================== M2: quasi-static vs in motion =====================
    # Direct signal of "almost zero displacement" = how much the OBJECT moved between the
    # two fragments (dist death A -> birth B). dist~0 => the ID split but the
    # object did NOT move (parked vehicle/flicker in place). large dist => it moved
    # (a moving occluded vehicle that reappears elsewhere). Corroborated with the track's
    # span (own mobility) and with its permanence (dwell).
    print("\nM2 — ¿la ruptura es de un vehículo con DESPLAZAMIENTO CASI NULO (parado)?")
    print("  distribución del desplazamiento de la ruptura (dist muerte->nacimiento):")
    buckets = [(0, 15), (15, 25), (25, 50), (50, 100), (100, 251)]
    for lo, hi in buckets:
        n = sum(1 for e in frags if lo <= e["dist"] < hi)
        print(f"    {lo:>3}-{hi:<3} px : {n:>3}  {'█'*n}")
    INPLACE = 25.0
    qs_events = [e for e in frags if e["dist"] < INPLACE]
    mv_events = [e for e in frags if e["dist"] >= INPLACE]
    print(f"  >>> EN SITIO (dist < {INPLACE:.0f}px, objeto no se movió) : {len(qs_events)}"
          f"   ({100*len(qs_events)/len(frags):.0f}%)")
    print(f"      DESPLAZADAS (objeto se movió)                : {len(mv_events)}"
          f"   ({100*len(mv_events)/len(frags):.0f}%)")
    # corroboration: are the tracks of the IN-PLACE breaks of low own mobility?
    qs_lowspan = sum(1 for e in qs_events if span(obs[e["a"]]) < 30)
    print(f"      corrobora: {qs_lowspan}/{len(qs_events)} EN-SITIO tienen track A de span<30px (baja movilidad)")
    from collections import Counter as C
    def region(v):
        gx = np.median([gc(o)[0] for o in v]); gy = np.median([gc(o)[1] for o in v])
        return ("SUR" if gy < 323 and gx < 600 else "OESTE" if gy < 323 else
                "ESTE" if gx < 600 else "NORTE")
    reg_qs = C(region(obs[e["a"]]) for e in qs_events)
    print(f"      región de las EN-SITIO: {dict(reg_qs)}")
    # M2b — inflation from quasi-static vehicles, INCLUDING short-gap flickers that the
    # 122 catalog (gap>=20f) excludes by design. A fragmented parked vehicle leaves
    # SEVERAL small-span tracks GROUPED at the same spot.
    stat = [(t, np.median([gc(o)[0] for o in v]), np.median([gc(o)[1] for o in v]))
            for t, v in counted.items() if span(v) < 25]
    clusters = []
    for t, x, y in stat:
        for c in clusters:
            if np.hypot(x - c["x"], y - c["y"]) < 40:
                c["ids"].append(t); break
        else:
            clusters.append({"x": x, "y": y, "ids": [t]})
    inflation = sum(len(c["ids"]) - 1 for c in clusters)
    print(f"  M2b inflación cuasi-estática (incluye flickers gap<20f):")
    print(f"     {len(stat)} tracks quietos (span<25px) en {len(clusters)} sitios "
          f"=> ~{inflation} IDs de exceso por vehículos parados fragmentados")
    for c in sorted(clusters, key=lambda c: -len(c["ids"]))[:6]:
        if len(c["ids"]) > 1:
            print(f"       sitio ({c['x']:.0f},{c['y']:.0f}) [{region([(0,c['x']-1,0,c['x']+1,c['y'])])}]"
                  f": {len(c['ids'])} IDs {c['ids']}")

    # ===================== M3: class blinking in general =====================
    print("\nM3 — blinkeo de clase en los 172 tracks contados (tamaño del problema de clase):")
    switches = {}
    multi = 0
    pair_trans = Counter()
    for t, v in counted.items():
        cs = [o[5] for o in v]
        sw = sum(1 for a, b in zip(cs, cs[1:]) if a != b)
        switches[t] = sw
        if len(set(cs)) > 1:
            multi += 1
            for a, b in zip(cs, cs[1:]):
                if a != b:
                    pair_trans[tuple(sorted((CLS.get(a), CLS.get(b))))] += 1
    n_sw = sum(1 for s in switches.values() if s > 0)
    print(f"  tracks con >=1 cambio de clase en su vida : {n_sw}/{len(counted)} "
          f"({100*n_sw/len(counted):.0f}%)")
    print(f"  tracks multi-clase                        : {multi}/{len(counted)}")
    tot_sw = sum(switches.values())
    print(f"  total de transiciones de clase            : {tot_sw} "
          f"(media {tot_sw/len(counted):.1f}/track; máx {max(switches.values())})")
    print(f"  pares de clase que oscilan                : {dict(pair_trans)}")

    # ===================== CROSS-CHECK: breakdown of the 122 breaks =====================
    print("\n" + "=" * 74)
    print("CRUCE — desglose de las rupturas (partición de los eventos)")
    moving = mv_events
    mv_classchg = [e for e in moving if e["cls_a"] != e["cls_b"]]
    mv_stable = [e for e in moving if e["cls_a"] == e["cls_b"]]
    print(f"  TOTAL eventos de fragmentación          : {len(frags)}")
    print(f"   1) EN SITIO (objeto no se movió, dist<25): {len(qs_events)}"
          f"   ({100*len(qs_events)/len(frags):.0f}%)  [parado/flicker; dedup trivial, no ruta]")
    qs_cc = sum(1 for e in qs_events if e['cls_a'] != e['cls_b'])
    print(f"        de ellas, además con cambio clase  : {qs_cc}")
    print(f"   2) DESPLAZADA + cambio de clase          : {len(mv_classchg)}"
          f"   ({100*len(mv_classchg)/len(frags):.0f}%)  [correlación, no causa]")
    print(f"   3) DESPLAZADA + clase estable            : {len(mv_stable)}"
          f"   ({100*len(mv_stable)/len(frags):.0f}%)  <-- objetivo real de reconexión por ruta")

    # relation to the excess IDs
    frag_birth_ids = {e["b"] for e in frags}
    qs_birth_ids = {e["b"] for e in qs_events}
    target_birth_ids = {e["b"] for e in mv_stable}
    print(f"\n  IDs-nacimiento de fragmentación (IDs redundantes)   : {len(frag_birth_ids)}")
    print(f"    de ellos en rupturas cuasi-estáticas              : {len(qs_birth_ids)}")
    print(f"    de ellos en rupturas en movimiento + clase estable: {len(target_birth_ids)}")
    print(f"  (exceso bruto de IDs vs GT = {len(counted)-GT_N})")
    print("=" * 74)


if __name__ == "__main__":
    main()

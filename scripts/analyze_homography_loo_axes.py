"""Decomposes the LEAVE-ONE-OUT error of the official homography into LONGITUDINAL (along the axis
of the approach edge, from the net) and LATERAL (perpendicular), PER APPROACH.

Why: position-driven DECOMPOSED fixes the lateral by LANE (discrete, SUMO centers) and only
pushes the longitudinal. → the LATERAL error of the homography does NOT reach SUMO. This measurement
puts a NUMBER on that gain: how much of H's error is lateral (removed by the per-lane discretization)
vs longitudinal (the part that actually affects `pos`). Same fit as the official H
(cv2.findHomography method=0, pixel→world). Read-only; writes no config or artifacts.
"""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
ARM_EDGE = {"sur": "E5", "norte": "E6", "este": "E3", "oeste": "E1"}
HOMOG_LOO_AGG = {"este": 0.79, "norte": 1.36, "sur": 1.76, "oeste": 4.61}  # aggregated LOO error per approach (m), official 15-point H


def edge_axes(net_path):
    root = ET.parse(net_path).getroot()
    central = next(j.get("id") for j in root.findall("junction") if j.get("type") == "traffic_light")
    axes = {}
    for e in root.findall("edge"):
        if e.get("to") != central or e.get("function") == "internal":
            continue
        arm = next((a for a, ed in ARM_EDGE.items() if ed == e.get("id")), None)
        if arm is None:
            continue
        ds = []
        for lane in e.findall("lane"):
            p = np.array([[float(c) for c in q.split(",")] for q in lane.get("shape").split()])
            ds.append(p[-1] - p[0])
        d = np.sum(ds, axis=0)
        axes[arm] = d / np.linalg.norm(d)
    return axes


def main():
    cp = pd.read_csv(ROOT / "verify_out/homography_gcp/control_points.csv")
    cp = cp[cp["available"]].reset_index(drop=True)
    px = cp[["pixel_x", "pixel_y"]].to_numpy(float)
    wd = cp[["world_x", "world_y"]].to_numpy(float)
    arms = cp["arm"].str.lower().to_numpy()
    axes = edge_axes(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml")

    # leave-one-out: refit without i, project pixel i, residual in world
    per_arm = {a: {"long": [], "lat": [], "tot": []} for a in ARM_EDGE}
    for i in range(len(cp)):
        mask = np.arange(len(cp)) != i
        H, _ = cv2.findHomography(px[mask], wd[mask], 0)
        h = np.array([px[i, 0], px[i, 1], 1.0])
        w = H @ h
        pred = w[:2] / w[2]
        res = pred - wd[i]                       # residual vector (world, m)
        arm = arms[i]
        if arm not in axes:
            continue
        u = axes[arm]                            # longitudinal (toward J5)
        v = np.array([-u[1], u[0]])              # lateral
        per_arm[arm]["long"].append(abs(res @ u))
        per_arm[arm]["lat"].append(abs(res @ v))
        per_arm[arm]["tot"].append(float(np.hypot(*res)))

    rms = lambda xs: float(np.sqrt(np.mean(np.square(xs)))) if xs else float("nan")
    print("Descomposición LOO del error de homografía por brazo (RMS, metros):")
    print(f"{'brazo':6s} {'N':>2s} | {'TOTAL':>6s} {'LONG':>6s} {'LAT':>6s} | "
          f"{'LAT elimina %':>12s} | {'LOO_agg(doc)':>12s}")
    print("-" * 70)
    all_long, all_lat, all_tot = [], [], []
    for arm in ["norte", "este", "oeste", "sur"]:
        d = per_arm[arm]
        if not d["tot"]:
            continue
        rl, rt, rtot = rms(d["long"]), rms(d["lat"]), rms(d["tot"])
        frac_lat = 100 * rt**2 / (rl**2 + rt**2) if (rl or rt) else float("nan")
        all_long += d["long"]; all_lat += d["lat"]; all_tot += d["tot"]
        print(f"{arm:6s} {len(d['tot']):2d} | {rtot:6.2f} {rl:6.2f} {rt:6.2f} | "
              f"{frac_lat:11.0f}% | {HOMOG_LOO_AGG[arm]:12.2f}")
    rl, rt, rtot = rms(all_long), rms(all_lat), rms(all_tot)
    print("-" * 70)
    print(f"{'TOTAL':6s} {len(all_tot):2d} | {rtot:6.2f} {rl:6.2f} {rt:6.2f} | "
          f"{100*rt**2/(rl**2+rt**2):11.0f}% |")
    print("\nLONG = error que SÍ afecta la `pos` longitudinal (lo que se empuja a SUMO).")
    print("LAT  = error que la discretización por CARRIL ELIMINA (SUMO centra en el lane).")
    print("LAT elimina % = fracción de la energía del error que el carril descarta.")
    print("Nota: TOTAL aquí es RMS de la norma del residuo; el LOO_agg de la doc usa la misma H")
    print("oficial (15 pts) — pequeñas diferencias por el refit LOO punto a punto, no material.")


if __name__ == "__main__":
    main()

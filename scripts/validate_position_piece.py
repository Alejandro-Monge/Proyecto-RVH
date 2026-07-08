"""Validation of the POSITION piece + the causal contract, with data.

Five blocks:
  A) Contract (clause i): the transform and the mapping are PURE lookups (check_pure_lookup).
  B) Contract (clause ii): the alpha-beta filter is backward-only (check_backward_only).
  C) Filter on a SYNTHETIC SIGNAL: at constant velocity it does NOT bias (lag≈0) and reduces noise; alpha
     sweep to justify the config value.
  D) Filter on REAL TRACKS: raw vs filtered longitudinal jitter; lag of a simple average.
  E) Longitudinal accuracy of the piece at the control points vs the long LOO (the position only
     carries the LONGITUDINAL error; the lateral is discarded by the lane).

Read only. It writes neither config nor artifacts.
"""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from digital_shadow.causal import check_backward_only, check_pure_lookup
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.position import (
    AlphaBetaFilter,
    load_alpha,
    load_homography,
    longitudinal_pos,
    project_to_world,
    raw_pos,
)

ARM_EDGE = {"sur": "E5", "norte": "E6", "este": "E3", "oeste": "E1"}


def accel_noise(s):
    if len(s) < 3:
        return np.nan
    d2 = s[2:] - 2 * s[1:-1] + s[:-2]
    return float(np.sqrt(np.mean(np.square(d2))))


def main():
    H = load_homography("rusia")
    lm = load_lane_map("rusia")
    alpha_cfg = load_alpha()

    # ---------- A) PURE transform / mapping ----------
    print("=== A) Contrato (i): lookups PUROS ===")
    pts = [(372.4, 108.0), (749.2, 469.8), (157.7, 308.4), (874.3, 191.9)]
    r = check_pure_lookup(lambda px: project_to_world(H, px), [(p,) for p in pts],
                          label="project_to_world")
    print(f"  project_to_world: PURO ({r['n']} inputs)")
    r = check_pure_lookup(lambda arm, px: raw_pos(H, lm.approach_axis[arm], px),
                          [("sur", (300.0, 90.0)), ("norte", (800.0, 400.0)),
                           ("este", (200.0, 350.0)), ("oeste", (850.0, 180.0))],
                          label="raw_pos")
    print(f"  raw_pos:          PURO ({r['n']} inputs)")
    r = check_pure_lookup(lm.target,
                          [("sur", "entry", 3), ("oeste", "exit", 2), ("este", "entry", 1)],
                          label="lane_map.target")
    print(f"  lane_map.target:  PURO ({r['n']} inputs)")

    # ---------- B) backward-only filter ----------
    print("\n=== B) Contrato (ii): filtro alpha-beta backward-only ===")
    stream = [(i * 0.0338, 10.0 + 4.0 * i * 0.0338 + (0.1 if i % 3 else -0.1)) for i in range(40)]
    r = check_backward_only(lambda: AlphaBetaFilter(alpha_cfg), stream, label="AlphaBetaFilter")
    print(f"  AlphaBetaFilter: BACKWARD-ONLY (prefijo reproduce traza, {r['n']} pasos)")

    # ---------- C) synthetic signal: lag and noise reduction ----------
    print("\n=== C) Filtro sobre señal sintética (vel. constante v=4 m/s + ruido σ=0.15 m) ===")
    rng = np.random.default_rng(7)
    dt = 0.0338
    t = np.arange(120) * dt
    true = 4.0 * t
    print(f"  {'alpha':>5s} {'beta':>5s} {'out_jit':>7s} {'lag_med':>7s}  (in_jit=0.150)")
    for alpha in [0.2, 0.3, 0.4, 0.5, 0.6]:
        f = AlphaBetaFilter(alpha)
        out = []
        for ti, tr in zip(t, true):
            f.observe(ti, tr + rng.normal(0, 0.15))
            out.append(f.value())
        out = np.array(out)
        warm = slice(20, None)  # after the transient
        out_jit = float(np.std(out[warm] - true[warm]))
        lag = float(np.median(out[warm] - true[warm]))
        mark = "  <- config" if abs(alpha - alpha_cfg) < 1e-9 else ""
        print(f"  {alpha:5.1f} {f.beta:5.2f} {out_jit:7.3f} {lag:7.3f}{mark}")
    print("  (vel. constante -> lag≈0 a todo alpha, propio del alpha-beta; ruido de salida < 0.15 entrada.)")

    # ---------- D) real tracks: raw vs filtered jitter ----------
    print("\n=== D) Tracks reales: jitter longitudinal crudo vs filtrado (alpha de config) ===")
    tracks = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/tracks_reconnected.csv")
    matches = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    import yaml
    with open(ROOT / "config/sites/rusia/zones.yaml") as fh:
        stop = {k: np.array(v, float) for k, v in yaml.safe_load(fh)["stop_lines"].items()}
    center = np.array([600.0, 323.0])
    by_track = {t: g.sort_values("frame_index") for t, g in tracks.groupby("track_id")}
    print(f"  {'brazo':6s} {'N':>3s} | {'acc_raw':>7s} {'acc_filt':>8s} {'reduc%':>6s} | {'lag140_simple':>13s}")
    agg = {}
    for _, m in matches.iterrows():
        arm = m["entry_arm"]
        g = by_track.get(int(m["track_id"]))
        if g is None or arm not in lm.approach_axis:
            continue
        img = g[["gx", "gy"]].to_numpy()
        ts = g["timestamp_s"].to_numpy()
        seg = stop[arm]
        v = seg[1] - seg[0]
        n = np.array([-v[1], v[0]], float)
        if n.dot(center - seg[0]) < 0:
            n = -n
        n /= np.linalg.norm(n)
        d = (img - seg[0]) @ n
        cr = [i for i in range(len(d) - 1) if d[i] <= 0 < d[i + 1]]
        end = cr[0] + 1 if cr else len(img)
        if end < 6:
            continue
        ax = lm.approach_axis[arm]
        s_raw = np.array([raw_pos(H, ax, tuple(p)) for p in img[:end]])
        f = AlphaBetaFilter(alpha_cfg)
        s_filt = []
        for ti, sr in zip(ts[:end], s_raw):
            f.observe(ti, sr)
            s_filt.append(f.value())
        s_filt = np.array(s_filt)
        a = agg.setdefault(arm, {"acc_raw": [], "acc_filt": [], "v": []})
        a["acc_raw"].append(accel_noise(s_raw))
        a["acc_filt"].append(accel_noise(s_filt))
        dtt = np.median(np.diff(ts[:end]))
        a["v"].append(np.median(np.abs(np.diff(s_raw))) / dtt if dtt > 0 else np.nan)
    for arm in ["norte", "este", "oeste", "sur"]:
        if arm not in agg:
            continue
        a = agg[arm]
        ar, af = np.nanmedian(a["acc_raw"]), np.nanmedian(a["acc_filt"])
        lag_simple = np.nanmedian(a["v"]) * 70 * 0.0338
        print(f"  {arm:6s} {len(a['acc_raw']):3d} | {ar:7.3f} {af:8.3f} {100*(1-af/ar):6.0f} | {lag_simple:13.1f}")
    print("  (acc=RMS 2a diferencia, m; el filtro reduce el jitter; lag140_simple=lo que sesgaría un")
    print("   promedio simple sobre el buffer de reconexión -> por eso NO se promedia sobre el buffer.)")

    # ---------- E) longitudinal accuracy vs LOO ----------
    print("\n=== E) Exactitud longitudinal de la pieza en los puntos de control (in-sample) ===")
    cp = pd.read_csv(ROOT / "verify_out/homography_gcp/control_points.csv")
    cp = cp[cp["available"]]
    print(f"  {'brazo':6s} {'N':>2s} | {'long_RMS(m)':>11s}  (= error que SÍ afecta la pos; el lateral lo descarta el carril)")
    for arm in ["norte", "este", "oeste", "sur"]:
        sub = cp[cp["arm"].str.lower() == arm]
        if sub.empty:
            continue
        ax = lm.approach_axis[arm]
        errs = []
        for _, row in sub.iterrows():
            pos_pred = raw_pos(H, ax, (row["pixel_x"], row["pixel_y"]))
            o, dd = ax.origin, ax.direction
            pos_true = (row["world_x"] - o[0]) * dd[0] + (row["world_y"] - o[1]) * dd[1]
            errs.append(pos_pred - min(max(pos_true, 0), ax.length))
        rms = float(np.sqrt(np.mean(np.square(errs))))
        print(f"  {arm:6s} {len(sub):2d} | {rms:11.2f}")
    print("  (consistente con la descomposición LOO long de scripts/analyze_homography_loo_axes.py.)")


if __name__ == "__main__":
    main()

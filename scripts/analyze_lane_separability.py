"""LANE SEPARABILITY analysis (reproducible evidence). Does NOT classify.

Measures whether the GT v2 lanes fall into separable groups, per approach, in the IMAGE PLANE (method:
image-plane bands), for the ENTRY lane. Two questions:

  1. SEPARABILITY vs MEASUREMENT POINT: do the lanes separate better
     RIGHT at the stop line than farther back in the approach (farther field)? Separability is measured
     by taking the ground contact at different distances from the stop line
     (0/40/80/120 px upstream) and with different windows at the line (±0/±3/±7 frames).
  2. SEPARABILITY vs approach error: the lane error rate is compared with the error of the
     APPROACH layer (baseline recall/precision ~5–15 %). INDIVIDUAL lane where
     its error is of the order of the approach error; GROUPED where it clearly exceeds it. (The decision
     threshold is external; here the table and confusion matrix are given.)

METRICS: d' = |Δμ|/combined_sd; gaussian error = Φ(−d'/2) (Bayes, optimal boundary, equal
priors); and non-parametric LOO = leave-one-out nearest-centroid (1-D, transverse image
coordinate), which counts real misclassifications and uses the real priors (more robust with small N).
Stated LIMITATION: small N (2–13/lane) → directional estimates, wide CIs.

INPUT: RECONNECTED tracks (default), GT v2, reconnected matches, stop lines from zones.yaml.
OUTPUT: only prints tables (writes no artifacts).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import norm

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))   # to import the sibling module, run as -m or directly
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import GROUND_X, GROUND_Y, add_ground_contact
import derive_track_events as dte   # load_stop_lines

ARMS = ["este", "norte", "oeste", "sur"]
LANE_M = 3.2   # approximate lane width (m), reference


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d
    return max(0.0, c - h), min(1.0, c + h)


def dprime(a, b):
    if len(a) < 2 or len(b) < 2:
        return float("nan")
    s = math.sqrt((np.var(a, ddof=1) + np.var(b, ddof=1)) / 2)
    return abs(np.mean(a) - np.mean(b)) / s if s > 0 else float("inf")


def loo_centroid(sub):
    """Leave-one-out nearest-centroid over `sub` (cols lane, s). Returns (errs, n, conf)."""
    sub = sub.reset_index(drop=True)
    n = len(sub)
    errs = 0
    conf = {}
    for i in range(n):
        tr = sub.drop(i)
        cent = tr.groupby("lane").s.mean()
        if cent.empty:
            continue
        pred = int((cent - sub.loc[i, "s"]).abs().idxmin())
        true = int(sub.loc[i, "lane"])
        conf[(true, pred)] = conf.get((true, pred), 0) + 1
        if pred != true:
            errs += 1
    return errs, n, conf


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="rusia")
    ap.add_argument("--tracks",
                    default="verify_out/events/rusia/reconnected/tracks_reconnected.csv")
    args = ap.parse_args()

    site = load_site_config(args.site)
    stop = {a: np.asarray(v, float) for a, v in dte.load_stop_lines(args.site).items()}
    tracks = pd.read_csv(ROOT / args.tracks)
    add_ground_contact(tracks)
    gt = pd.read_csv(ROOT / "data" / "ground_truth" / args.site /
                     f"ground_truth_{args.site}_v2_carril_processed.csv").set_index("event_id")
    matches = pd.read_csv(ROOT / "verify_out" / "events" / args.site / "reconnected" / "matches.csv")
    by_track = {t: g.sort_values("frame_index") for t, g in tracks.groupby("track_id")}

    def approach_axis(arm):
        p1, p2 = stop[arm][0], stop[arm][1]
        u = (p2 - p1) / np.linalg.norm(p2 - p1)            # transverse axis (along the stop line)
        n = np.array([-(p2 - p1)[1], (p2 - p1)[0]], float)
        n /= np.linalg.norm(n)
        center = np.array([site.frame_width / 2, site.frame_height / 2])
        if n @ (center - p1) > 0:                          # n points to the center → approach = -n
            n = -n
        return p1, u, n                                    # n = normal toward the approach side (+)

    def s_at(tid, arm, dist_px, win):
        """Transverse coordinate of the ground contact `dist_px` upstream of the stop line (±win frames)."""
        g = by_track.get(tid)
        if g is None:
            return None
        p1, u, n_app = approach_axis(arm)
        pts = g[[GROUND_X, GROUND_Y]].to_numpy()
        d_app = (pts - p1) @ n_app                         # >0 approach, 0 at the stop line, <0 inside
        # target frame: the nearest to dist_px on the approach SIDE
        cand = np.where(d_app >= -5)[0]                    # up to the stop line (incl. small margin)
        if len(cand) == 0:
            return None
        j = cand[np.argmin(np.abs(d_app[cand] - dist_px))]
        if abs(d_app[j] - dist_px) > 25:                   # did not reach that distance upstream
            return None
        lo, hi = max(0, j - win), min(len(g), j + win + 1)
        pt = np.median(pts[lo:hi], axis=0)
        return float((pt - p1) @ u)

    def collect(dist_px, win):
        rows = []
        for _, m in matches.iterrows():
            ev = gt.loc[m.event_id]
            s = s_at(int(m.track_id), ev.entry_arm, dist_px, win)
            if s is not None:
                rows.append(dict(arm=ev.entry_arm, lane=int(ev.entry_lane), s=s))
        return pd.DataFrame(rows)

    # --- 1. Separability vs MEASUREMENT POINT ---
    points = [("pare ±3 (actual)", 0, 3), ("pare exacto ±0", 0, 0), ("pare ±7", 0, 7),
              ("atras 40px", 40, 2), ("atras 80px", 80, 2), ("atras 120px", 120, 2)]
    print("=" * 92)
    print("1) SEPARABILIDAD de carril de ENTRADA (plano imagen) vs PUNTO DE MEDICIÓN en la aproximación")
    print("   d' por par adyacente y LOO por brazo. Foco: oeste 2-3, sur 1-2 y 2-3 (marginales).")
    print("=" * 92)
    for label, dist, win in points:
        df = collect(dist, win)
        print(f"\n--- punto = {label}  (N total={len(df)}) ---")
        hdr = f"{'brazo':6} " + " ".join(f"{'d'+str(L)+str(L+1):>8}" for L in (1, 2)) + \
              f" {'LOO_err':>9} {'IC95':>14}"
        print(hdr)
        for arm in ARMS:
            sub = df[df.arm == arm]
            lanes = sorted(sub.lane.unique())
            dvals = {}
            for L in (1, 2):
                a = sub[sub.lane == L].s.to_numpy(); b = sub[sub.lane == L + 1].s.to_numpy()
                dvals[L] = dprime(a, b)
            e, n, _ = loo_centroid(sub[["lane", "s"]]) if len(sub) else (0, 0, {})
            lo, hi = wilson(e, n)
            dtxt = " ".join(f"{dvals[L]:>8.2f}" if not math.isnan(dvals[L]) else f"{'·':>8}" for L in (1, 2))
            ic = f"[{lo*100:.0f},{hi*100:.0f}]%" if n else "—"
            print(f"{arm:6} {dtxt} {e}/{n:<2}={e/n*100 if n else 0:4.0f}% {ic:>14}")

    # --- 2. Detail at the STOP LINE: d' -> gaussian error + LOO + confusion matrix + anchoring to approach ---
    df0 = collect(0, 3)
    print("\n" + "=" * 92)
    print("2) DETALLE en la línea de pare (±3): d'→error gaussiano, y matriz de confusión LOO por brazo")
    print("=" * 92)
    print(f"{'brazo':6} {'par':4} {'n_a':>3} {'n_b':>3} {'d_prime':>8} {'err_gauss':>10}")
    for arm in ARMS:
        sub = df0[df0.arm == arm]; lanes = sorted(sub.lane.unique())
        for k in range(len(lanes) - 1):
            a = sub[sub.lane == lanes[k]].s.to_numpy(); b = sub[sub.lane == lanes[k + 1]].s.to_numpy()
            d = dprime(a, b); err = norm.cdf(-d / 2) if math.isfinite(d) else float("nan")
            print(f"{arm:6} {lanes[k]}-{lanes[k+1]:<2} {len(a):>3} {len(b):>3} {d:>8.2f} {err*100:>9.2f}%")
    print("\nMatriz de confusión LOO por brazo (true→pred : conteo), en la pare:")
    for arm in ARMS:
        sub = df0[df0.arm == arm]
        e, n, conf = loo_centroid(sub[["lane", "s"]])
        lo, hi = wilson(e, n)
        cells = ", ".join(f"{t}→{p}:{c}" for (t, p), c in sorted(conf.items()))
        print(f"  {arm:6} LOO={e}/{n}={e/n*100:4.0f}% IC95[{lo*100:.0f},{hi*100:.0f}]%  | {cells}")

    # --- 3. Anchoring to the APPROACH error (no magic threshold) ---
    print("\n" + "=" * 92)
    print("3) ANCLAJE al error de la capa de BRAZO (individual si el error de")
    print("   carril es del orden del de brazo; agrupado si lo excede claramente)")
    print("=" * 92)
    rep = pd.read_csv(ROOT / "verify_out" / "events" / args.site /
                      "reconnected" / "baseline_por_brazo_reconnected.csv").set_index("brazo_entrada")
    print("Error de BRAZO por brazo (reconectado): 1-recall (omisión) y 1-precisión (FP):")
    for arm in ARMS:
        r = rep.loc[arm]
        print(f"  {arm:6} 1-recall={ (1-r.recall)*100:4.1f}%  1-precision={(1-r.precision)*100:4.1f}%")
    print("  (banda de referencia de error de brazo ≈ 5–16 %)")


if __name__ == "__main__":
    main()

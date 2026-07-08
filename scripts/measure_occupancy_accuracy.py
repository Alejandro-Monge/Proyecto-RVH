"""TUNING instrument for the approach occupancy layer against its GT — MEASUREMENT LAYER.

NOT production nor a paper metric: it calibrates (θ = dwell, g = gap merge) and
diagnoses the occupancy layer (`digital_shadow/occupancy/`) against the occupancy GT.
In-sample over the single 180 s clip — declared; nothing here migrates to results.
Two evaluation levels:

  TIER A (tuning objective): COUNT series per zone n_sys(t) vs n_gt(t), without identity.
    θ and g are swept JOINTLY against this series (the magnitude the layer delivers), not
    against the matched events (that would optimize matching artifacts). Choice by
    PLATEAU, not peak (n=77, small sample).
  TIER B (diagnostic, with identity): MONOTONE alignment by EXIT order per zone
    (Needleman-Wunsch, dual cost |Δexit|+|Δentry| over NON-censored anchors, gaps
    = FN/FP). Chosen with data: plain overlap drops to 54-84 % identity with real
    queues; the monotone alignment gives ~100 % even with 30 % losses (simulation
    over the 77 real GT windows). It produces losses/ghosts/fragments
    per zone and the distributions Δentry (nose→contact offset, known systematic
    bias) and Δexit (verification of the crossings-GT anchor: if the GT stop line was
    annotated with another criterion, it shows up here as a duration bias).

EDGE-CASE CONVENTIONS (general rule: a censored edge never generates
an error term):
  - entry censoring: GT interval [0, exit]; out of the Δentry statistics.
  - exit censoring (non-crossers): [entry, 180]; out of the Δexit ones.
  - `na` (ev77-79, never cross their lane detector): no lane interval, they count
    only as a DECLARED UNDER-COUNT of the south approach aggregate (no temporal anchor).
  - crosser exit = `entry_time_s` of the crossings GT (stop-line crossing).
  - APPROACH level: GT = Σ lane intervals + sin_carril with times. TWO things are
    compared: (i) Σ system lane-zone counts vs that series (clean) and (ii) the
    approach POLYGON count vs that series (informative: it carries a positive structural
    bias because the approach polygon is deeper than the lane detectors — do NOT
    tune θ to kill it).

Reads tracks from the production online tracker (verify_out/tracks_rusia.csv) + ground contact
from the contract + parking-lot exclusion. Writes ONLY to verify_out/occupancy_accuracy/
(with an md5 provenance stamp). It does not import the causal pipeline nor touch pinned inputs.

Usage:
  python scripts/measure_occupancy_accuracy.py --sweep            # θ×g sweep (Tier A)
  python scripts/measure_occupancy_accuracy.py --theta 0.5 --gap 1.0   # Tier A+B at one point
"""
from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from digital_shadow.geometry.ground_contact import add_ground_contact  # noqa: E402
from digital_shadow.occupancy.zones import (classify_points, load_approach_zones,  # noqa: E402
                                            _inside)
import derive_track_events as dte  # noqa: E402

GT_OCC = ROOT / "data/ground_truth/rusia/ground_truth_occupancy_rusia.csv"
GT_CRU = ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv"
TRACKS = ROOT / "verify_out/tracks_rusia.csv"
ZONES_YAML = ROOT / "config/sites/rusia/approach_zones.yaml"
OUT_DIR = ROOT / "verify_out/occupancy_accuracy"
CLIP_END = 180.0
FRAME_DT = 12.0 / 355.0          # video step (≈33.8 ms); the series is evaluated at this step
GAP_PENALTY_S = 3.0              # aligner gap penalty (FN/FP); CLI --align-gap
EDGE_EPS = 0.5                   # margin (s) to declare censoring at the clip edges


# --------------------------------------------------------------------------- #
# GT: intervals per zone (pure functions, tested in tests/test_occupancy_accuracy.py)
# --------------------------------------------------------------------------- #
def build_gt_intervals(occ: pd.DataFrame, cruces: pd.DataFrame, clip_end: float = CLIP_END):
    """Returns (intervals df, sin_ancla df). Intervals: ev, arm, lane ('1'/'2'/'3'/
    'sin_carril'), t0, t1, cens_in, cens_out. sin_ancla: `na` rows (declaration only)."""
    t_pare = dict(zip(cruces.event_id, cruces.entry_time_s))
    rows, na_rows = [], []
    for r in occ.itertuples():
        enter = str(r.zone_enter_time_s)
        if enter == "na":                          # never crosses its lane detector
            na_rows.append(dict(ev=r.event_id, arm=r.entry_arm))
            continue
        cens_in = r.entrada_censurada == "si"
        if not cens_in and pd.isna(r.zone_enter_time_s):
            raise ValueError(f"ev{r.event_id}: zone_enter vacío sin censura (GT incompleto)")
        t0 = 0.0 if cens_in else float(r.zone_enter_time_s)
        if r.crosses == "si":
            t1, cens_out = float(t_pare[r.event_id]), False
        else:
            cens_out = pd.isna(r.zone_exit_time_s) or str(r.zone_exit_time_s) == "na"
            t1 = clip_end if cens_out else float(r.zone_exit_time_s)
        rows.append(dict(ev=r.event_id, arm=r.entry_arm, lane=str(r.entry_lane),
                         t0=t0, t1=t1, cens_in=cens_in, cens_out=cens_out))
    return pd.DataFrame(rows), pd.DataFrame(na_rows)


# --------------------------------------------------------------------------- #
# System: intervals per track and zone (pure)
# --------------------------------------------------------------------------- #
def presence_intervals(ts: np.ndarray, max_hole: float = 1.5 * FRAME_DT):
    """Presence timestamps (sorted) → intervals [t0, t1] cutting at gaps."""
    if len(ts) == 0:
        return []
    cuts = np.where(np.diff(ts) > max_hole)[0]
    starts = np.concatenate([[0], cuts + 1])
    ends = np.concatenate([cuts, [len(ts) - 1]])
    return [(float(ts[a]), float(ts[b])) for a, b in zip(starts, ends)]


def merge_intervals(intervals, gap: float):
    """Merges intervals (same track/zone) with gap ≤ gap."""
    if not intervals:
        return []
    ivs = sorted(intervals)
    out = [list(ivs[0])]
    for t0, t1 in ivs[1:]:
        if t0 - out[-1][1] <= gap:
            out[-1][1] = max(out[-1][1], t1)
        else:
            out.append([t0, t1])
    return [(a, b) for a, b in out]


def apply_threshold(intervals, theta: float):
    """Discards intervals with dwell < theta."""
    return [(a, b) for a, b in intervals if (b - a) >= theta]


def count_series(intervals, grid: np.ndarray) -> np.ndarray:
    """Count per grid instant (inclusive edges)."""
    n = np.zeros(len(grid), dtype=int)
    for t0, t1 in intervals:
        n[(grid >= t0) & (grid <= t1)] += 1
    return n


# --------------------------------------------------------------------------- #
# Tier B: monotone alignment by exit with gaps (pure)
# --------------------------------------------------------------------------- #
def _pair_cost(g, s, big: float = 1e6):
    """Dual cost over NON-censored anchors; no common anchor → prohibitive."""
    terms = []
    if not g["cens_out"] and not s["cens_out"]:
        terms.append(abs(g["t1"] - s["t1"]))
    if not g["cens_in"] and not s["cens_in"]:
        terms.append(abs(g["t0"] - s["t0"]))
    return float(np.mean(terms)) if terms else big


def align_monotone(gt_events, sys_events, gap_penalty: float = GAP_PENALTY_S):
    """Needleman-Wunsch over sequences ordered by exit. Returns
    (pairs [(i_gt, j_sys)], gt_sin_match, sys_sin_match) with indices of the GIVEN LISTS."""
    # Order by exit; exit ties (e.g. both censored at end of clip) are broken
    # by entry — otherwise monotonicity would forbid the correct pairing.
    gt_o = sorted(range(len(gt_events)), key=lambda i: (gt_events[i]["t1"], gt_events[i]["t0"]))
    sy_o = sorted(range(len(sys_events)), key=lambda j: (sys_events[j]["t1"], sys_events[j]["t0"]))
    n, m = len(gt_o), len(sy_o)
    D = np.full((n + 1, m + 1), np.inf)
    D[0, :] = np.arange(m + 1) * gap_penalty
    D[:, 0] = np.arange(n + 1) * gap_penalty
    P = np.zeros((n + 1, m + 1), dtype=int)
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            c = _pair_cost(gt_events[gt_o[i - 1]], sys_events[sy_o[j - 1]])
            opts = ((D[i - 1, j - 1] + c, 0), (D[i - 1, j] + gap_penalty, 1),
                    (D[i, j - 1] + gap_penalty, 2))
            D[i, j], P[i, j] = min(opts)
    pairs, i, j = [], n, m
    while i > 0 and j > 0:
        if P[i, j] == 0:
            pairs.append((gt_o[i - 1], sy_o[j - 1])); i, j = i - 1, j - 1
        elif P[i, j] == 1:
            i -= 1
        else:
            j -= 1
    matched_g = {a for a, _ in pairs}
    matched_s = {b for _, b in pairs}
    return (pairs, [i for i in range(n) if i not in matched_g],
            [j for j in range(m) if j not in matched_s])


# --------------------------------------------------------------------------- #
# Construction of the system side (I/O)
# --------------------------------------------------------------------------- #
def build_system_presence():
    """Presence per (zone, track): dict zone -> {track_id: [raw intervals]}.
    Zones: ('arm', approach) and ('lane', approach, k). Ground contact + contract exclusion."""
    zones = load_approach_zones("rusia")
    excl, _, _ = dte.load_exclusion_polygons("rusia")
    tr = pd.read_csv(TRACKS)
    add_ground_contact(tr)
    pts = tr[["gx", "gy"]].to_numpy(float)
    keep = np.ones(len(tr), bool)
    for poly in excl.values():
        keep &= ~_inside(pts, np.asarray(poly, float))
    tr = tr[keep].reset_index(drop=True)
    pts = tr[["gx", "gy"]].to_numpy(float)
    # SINGLE SOURCE of membership: the lane mask is intersected with the
    # approach (lane ⊂ approach, loader guarantee) — empirical equivalence
    # 0/106480 over the reference segment.
    arm_col, lane_col = classify_points(pts, zones)   # excl already applied above
    presence = {}
    for arm, az in zones.arms.items():
        masks = {("arm", arm): arm_col == arm}
        for k in az.lanes:
            masks[("lane", arm, k)] = (arm_col == arm) & (lane_col == k)
        for zone, mask in masks.items():
            sub = tr[mask]
            presence[zone] = {tid: presence_intervals(np.sort(g.timestamp_s.to_numpy(float)))
                              for tid, g in sub.groupby("track_id")}
    return presence


def system_events(presence_zone: dict, theta: float, gap: float):
    """Final intervals per zone (g merge per track + θ threshold), with edge censoring."""
    events = []
    for tid, ivs in presence_zone.items():
        for t0, t1 in apply_threshold(merge_intervals(ivs, gap), theta):
            events.append(dict(track=tid, t0=t0, t1=t1,
                               cens_in=t0 <= EDGE_EPS, cens_out=t1 >= CLIP_END - EDGE_EPS))
    return events


# --------------------------------------------------------------------------- #
# Tier A / Tier B
# --------------------------------------------------------------------------- #
def gt_series_by_zone(iv: pd.DataFrame, grid: np.ndarray):
    """GT series: per lane zone and approach aggregate (Σ lanes + sin_carril with t)."""
    series = {}
    for arm in iv.arm.unique():
        g_arm = iv[iv.arm == arm]
        series[("arm", arm)] = count_series(list(zip(g_arm.t0, g_arm.t1)), grid)
        for lane in sorted(g_arm.lane.unique()):
            if lane == "sin_carril":
                continue
            g = g_arm[g_arm.lane == lane]
            series[("lane", arm, int(lane))] = count_series(list(zip(g.t0, g.t1)), grid)
    return series


def tier_a(presence, gt_series, grid, theta, gap):
    """MAE / signed error / exact frames per zone; lane and approach aggregates."""
    rows = []
    lane_zones = [z for z in gt_series if z[0] == "lane"]
    sys_lane_sum = {}
    for zone in lane_zones:
        ev = system_events(presence.get(zone, {}), theta, gap)
        ns = count_series([(e["t0"], e["t1"]) for e in ev], grid)
        sys_lane_sum.setdefault(zone[1], np.zeros(len(grid), int))
        sys_lane_sum[zone[1]] += ns
        ng = gt_series[zone]
        d = ns.astype(float) - ng
        rows.append(dict(zona=f"{zone[1]}/c{zone[2]}", nivel="carril", mae=np.abs(d).mean(),
                         bias=d.mean(), exact=float((d == 0).mean()), n_gt=int(ng.max())))
    for arm, ns in sys_lane_sum.items():                       # Σ system lanes vs approach GT
        if ("arm", arm) not in gt_series:
            continue
        d = ns.astype(float) - gt_series[("arm", arm)]
        rows.append(dict(zona=f"{arm}/Σcarriles", nivel="brazo_suma", mae=np.abs(d).mean(),
                         bias=d.mean(), exact=float((d == 0).mean()),
                         n_gt=int(gt_series[("arm", arm)].max())))
    # sorted(): iterating a raw set ordered the "polígono" rows by the process hash
    # seed -> tier_a_zonas.csv not byte-for-byte reproducible.
    for arm in sorted({z[1] for z in gt_series if z[0] == "arm"}):  # approach polygon (informative)
        ev = system_events(presence.get(("arm", arm), {}), theta, gap)
        ns = count_series([(e["t0"], e["t1"]) for e in ev], grid)
        d = ns.astype(float) - gt_series[("arm", arm)]
        rows.append(dict(zona=f"{arm}/polígono", nivel="brazo_poligono", mae=np.abs(d).mean(),
                         bias=d.mean(), exact=float((d == 0).mean()),
                         n_gt=int(gt_series[("arm", arm)].max())))
    return pd.DataFrame(rows)


def tier_b(presence, iv: pd.DataFrame, theta, gap, align_gap):
    """Alignment per lane zone → matches + losses/ghosts/fragments."""
    matches, losses, ghosts = [], [], []
    for (arm, lane), g in iv[iv.lane != "sin_carril"].groupby(["arm", "lane"]):
        zone = ("lane", arm, int(lane))
        gt_ev = g.to_dict("records")
        sy_ev = system_events(presence.get(zone, {}), theta, gap)
        pairs, gt_um, sy_um = align_monotone(gt_ev, sy_ev, align_gap)
        for i, j in pairs:
            ge, se = gt_ev[i], sy_ev[j]
            matches.append(dict(
                zona=f"{arm}/c{lane}", ev=ge["ev"], track=se["track"],
                d_entrada=(se["t0"] - ge["t0"]) if not (ge["cens_in"] or se["cens_in"]) else np.nan,
                d_salida=(se["t1"] - ge["t1"]) if not (ge["cens_out"] or se["cens_out"]) else np.nan,
                gt=f"[{ge['t0']:.1f},{ge['t1']:.1f}]", sys=f"[{se['t0']:.1f},{se['t1']:.1f}]"))
        for i in gt_um:
            ge = gt_ev[i]
            losses.append(dict(zona=f"{arm}/c{lane}", ev=ge["ev"],
                               gt=f"[{ge['t0']:.1f},{ge['t1']:.1f}]"))
        for j in sy_um:
            se = sy_ev[j]
            # fragment if its midpoint falls inside some GT interval of the zone
            mid = 0.5 * (se["t0"] + se["t1"])
            frag = any(ge["t0"] <= mid <= ge["t1"] for ge in gt_ev)
            ghosts.append(dict(zona=f"{arm}/c{lane}", track=se["track"],
                               tipo="fragmento" if frag else "fantasma",
                               sys=f"[{se['t0']:.1f},{se['t1']:.1f}]"))
    return pd.DataFrame(matches), pd.DataFrame(losses), pd.DataFrame(ghosts)


# --------------------------------------------------------------------------- #
def provenance():
    lines = ["instrumento: measure_occupancy_accuracy.py (capa de medición, tuneo in-sample; NO paper)"]
    for p in [GT_OCC, GT_CRU, TRACKS, ZONES_YAML, ROOT / "config/sites/rusia/zones.yaml"]:
        lines.append(f"  {p.relative_to(ROOT)} = {hashlib.md5(p.read_bytes()).hexdigest()}")
    return "\n".join(lines) + "\n"


def load_measurement_defaults():
    """Working point (θ, g) from approach_zones.yaml: measurement (the rationale
    is in the yaml itself). CLI only for override/experiments."""
    import yaml
    with open(ZONES_YAML, encoding="utf-8") as fh:
        m = (yaml.safe_load(fh) or {}).get("measurement") or {}
    return float(m.get("theta_s", 0.0)), float(m.get("merge_gap_s", 0.0))


def main():
    theta_cfg, gap_cfg = load_measurement_defaults()
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--theta", type=float, default=theta_cfg,
                    help=f"permanencia mínima en zona (s); default de config: {theta_cfg}")
    ap.add_argument("--gap", type=float, default=gap_cfg,
                    help=f"fusión de huecos por track (s); default de config: {gap_cfg}")
    ap.add_argument("--align-gap", type=float, default=GAP_PENALTY_S)
    ap.add_argument("--sweep", action="store_true", help="barrido conjunto θ×g (Tier A)")
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    occ = pd.read_csv(GT_OCC, dtype={"zone_enter_time_s": str, "zone_exit_time_s": str})
    for c in ("zone_enter_time_s", "zone_exit_time_s"):       # 'na' survives; numbers parse
        occ[c] = occ[c].map(lambda v: v if (isinstance(v, str) and v == "na")
                            else (float(v) if pd.notna(v) and str(v) != "nan" else np.nan))
    cruces = pd.read_csv(GT_CRU)
    iv, sin_ancla = build_gt_intervals(occ, cruces)
    grid = np.arange(0.0, CLIP_END + 1e-9, FRAME_DT)
    gts = gt_series_by_zone(iv, grid)
    presence = build_system_presence()

    print(f"GT: {len(iv)} intervalos ({int(iv.cens_in.sum())} censura entrada, "
          f"{int(iv.cens_out.sum())} censura salida) · sin ancla (na): {len(sin_ancla)} "
          f"-> sub-conteo GT declarado en brazo {sorted(set(sin_ancla.arm)) if len(sin_ancla) else '-'}")

    if args.sweep:
        rows = []
        for theta in np.arange(0.0, 3.01, 0.25):
            for gap in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0):
                ta = tier_a(presence, gts, grid, theta, gap)
                lanes = ta[ta.nivel == "carril"]
                rows.append(dict(theta=round(theta, 2), gap=gap,
                                 mae_carriles=lanes.mae.mean(), bias_carriles=lanes.bias.mean(),
                                 exact_carriles=lanes.exact.mean()))
        sw = pd.DataFrame(rows)
        sw.to_csv(OUT_DIR / "sweep_theta_g.csv", index=False)
        piv = sw.pivot(index="theta", columns="gap", values="mae_carriles")
        print("\nMAE medio de zonas de carril (barrido θ filas × g columnas):")
        print(piv.round(4).to_string())
        (OUT_DIR / "provenance.txt").write_text(provenance())
        print(f"\n[ok] {OUT_DIR.relative_to(ROOT)}/sweep_theta_g.csv (+provenance)")
        return

    ta = tier_a(presence, gts, grid, args.theta, args.gap)
    # CONTRACTUAL file order (by zone): redundant with the sorted() in
    # tier_a, guarantees the contract even if an internal iteration changes in the future
    ta = ta.sort_values("zona", kind="stable").reset_index(drop=True)
    print(f"\n--- TIER A (θ={args.theta}, g={args.gap}) ---")
    print(ta.round(4).to_string(index=False))
    ta.to_csv(OUT_DIR / "tier_a_zonas.csv", index=False)

    m, l, gh = tier_b(presence, iv, args.theta, args.gap, args.align_gap)
    m.to_csv(OUT_DIR / "matches_eventos.csv", index=False)
    l.to_csv(OUT_DIR / "perdidas.csv", index=False)
    gh.to_csv(OUT_DIR / "fantasmas_fragmentos.csv", index=False)
    print(f"\n--- TIER B: {len(m)} matches · {len(l)} pérdidas · "
          f"{(gh.tipo == 'fragmento').sum() if len(gh) else 0} fragmentos · "
          f"{(gh.tipo == 'fantasma').sum() if len(gh) else 0} fantasmas ---")
    if len(l):
        print("pérdidas:\n" + l.to_string(index=False))
    if len(gh):
        print(gh.groupby(["zona", "tipo"]).size().to_string())
    arm_of = {f"{a}/c{l_}": a for a in iv.arm.unique() for l_ in (1, 2, 3)}
    m["arm"] = m.zona.map(arm_of)
    print("\nΔentrada por brazo (sistema−GT, s; desfase nariz→contacto, anclas no censuradas):")
    print(m.groupby("arm").d_entrada.describe(percentiles=[.1, .5, .9])
          [["count", "10%", "50%", "90%"]].round(2).to_string())
    print("\nΔsalida por brazo (verificación del ancla del GT de cruces):")
    print(m.groupby("arm").d_salida.describe(percentiles=[.1, .5, .9])
          [["count", "10%", "50%", "90%"]].round(2).to_string())
    (OUT_DIR / "provenance.txt").write_text(provenance())
    print(f"\n[ok] {OUT_DIR.relative_to(ROOT)}: tier_a_zonas, matches_eventos, perdidas, "
          f"fantasmas_fragmentos, provenance")


if __name__ == "__main__":
    main()

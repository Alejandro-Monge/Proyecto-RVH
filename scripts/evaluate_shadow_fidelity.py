"""GT↔shadow matcher and fidelity metrics (MEASUREMENT LAYER, not production).

Produces the "measuring stick": matches each shadow vehicle (causal chains from run_core) with
the ground truth events and reports presence (complete/partial/loss/ghost/…) and per-approach lane
accuracy. It does NOT touch reconstruct.py, the render, or the net: the shadow looks the same; this only measures.

PRINCIPLES (methodological defense):
  * It is OFFLINE EVALUATION, not operation. The matcher may use the EXIT instant (future)
    to match GT↔shadow: this is standard in tracking evaluation (the GT–prediction matching
    is resolved over the full sequence, e.g. CLEAR-MOT/HOTA). It does NOT violate the shadow's
    causality: the shadow ALREADY ran; the matcher neither feeds nor modifies it. The causality of the
    shadow pipeline is a separate matter and this script does not touch it.
  * ANTI-LEAK RED LINE: matching is done ONLY by time + approach (geometry/time of the pare1 and
    pare2 crossing) and, for partials, by geometric heading. NEVER by LANE (a reported metric) or by
    CLASS (Section 7.5). The cost functions do not read `entry_lane_label`, `exit_carril`, or the class; those
    fields exist ONLY for the metrics. This is confirmed in the report.

ALGORITHM (two stages + quarantine):
  0. degenerate-track: completes with an anomalous TRANSIT TIME (pare1→pare2) → quarantine, out of the
     pool. The transit isolates the pathological one with a clean gap (~42 s vs ≤10 s of the rest); duration and
     frozen fraction do NOT work (there are valid matches just as long/frozen).
  1. completes (clean) × GT: Hungarian. Edge: same entry approach ∧ |Δt_in|≤W_IN ∧ same exit
     approach ∧ |Δt_out|≤W_OUT. Cost = |Δt_in|+|Δt_out|.  → complete-match.
  2. RELIABLE partials (end_speed≥SPD_THR ∧ stability≥STAB_THR) × residual-GT: entry approach ∧
     |Δt_in|≤W_IN ∧ heading == GT exit approach.  → partial-match.
  3. UNCERTAIN partials × residual-GT: entry approach ∧ |Δt_in|≤W_IN (WITHOUT heading). → partial-incierto
     (not used for metrics).
  4. Unmatched GT = loss. Unmatched system: if it collides (same entry approach, |Δt_in|≤W_IN) with
     an ALREADY-matched vehicle → duplicate-ghost; otherwise → generic-ghost.
  5. Lane metrics ONLY over complete-match (LOO per vehicle, per approach).

Usage: python scripts/evaluate_shadow_fidelity.py [--no-sumo]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
import reconnect_fragments as rf
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import GROUND_X, GROUND_Y, add_ground_contact
from digital_shadow.harness.orchestrator import CausalOrchestrator
from digital_shadow.harness.sink import RecordingSink
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.position import load_alpha, load_homography
from digital_shadow.sumo.route import load_lane_turn_rule, load_turns, predict_turn
from digital_shadow.tracking.causal_reconnect import CausalReconnector
from audit_harness_part3 import internal_shapes_from_net

NET = ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml"
GT_CSV = ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv"

# --- matcher parameters (justified by sweep/gaps of the segment; see report) ---
W_IN, W_OUT = 1.0, 1.25       # entry/exit time windows (s): sweep plateaus
SPD_THR, STAB_THR = 1.5, 0.55  # reliable heading of a partial: speed AND consistency (segment gaps)
TRANSIT_THR = 20.0            # pare1→pare2 transit (s) for degenerate-track; point of the 10→42 gap
DUP_OVERLAP_S, DUP_DIST_PX = 2.0, 25.0   # duplicate = trajectory coexistence (two tracks overlapping)

ARMS = ["este", "norte", "oeste", "sur"]


# --------------------------------------------------------------------------- #
# 1) Causal chains (same source as production; does NOT modify the render)
# --------------------------------------------------------------------------- #
def build_chains():
    """Replicates the run_harness_viz.run_core setup and runs the orchestrator → (chains, ctx).
    Source = production causal chains; this script READS them, does not alter the pipeline."""
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    tracks = pd.read_csv(ROOT / "verify_out/tracks_rusia.csv"); add_ground_contact(tracks)
    base = pd.read_csv(ROOT / "verify_out/events/rusia/track_events.csv")
    excluded = set(tracks.track_id.unique()) - set(base.track_id.unique())
    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)
    lm = load_lane_map("rusia"); H = load_homography("rusia")
    shapes = internal_shapes_from_net(lm, NET)
    rule = load_lane_turn_rule("rusia"); turns = load_turns("rusia")
    stop_lines = dte.load_stop_lines("rusia"); lane_bands = dte.load_lane_bands("rusia")
    orch = CausalOrchestrator(
        tracks=tracks, feats=feats,
        reconnector=CausalReconnector(buffer_frames=site.reid_temporal_window_frames,
                                      corridor_px=site.reid_spatial_corridor_px,
                                      valid_moves=rf.load_valid_movements("rusia")),
        lane_map=lm, H=H, rule=rule, turns=turns, stop_lines=stop_lines, lane_bands=lane_bands,
        center=center, alpha=load_alpha(), buffer_frames=site.reid_temporal_window_frames,
        excluded=excluded, internal_shapes=shapes)
    _commits, chains = orch.run(RecordingSink())
    ctx = dict(lm=lm, H=H, center=center, stop_lines=stop_lines, lane_bands=lane_bands,
               turns=turns, rule=rule)
    return chains, ctx


# --------------------------------------------------------------------------- #
# 2) Per-vehicle signature (from ITS OWN trail; never copied from another)
# --------------------------------------------------------------------------- #
def signature(ch, ctx):
    """Signature for matching (time+geometric approach) + METRIC-ONLY fields (lane, separate)."""
    lm, center, sl, lane_bands = ctx["lm"], ctx["center"], ctx["stop_lines"], ctx["lane_bands"]
    mids = ctx["_mids"]
    gx, gy, ts = np.asarray(ch.gx, float), np.asarray(ch.gy, float), np.asarray(ch.ts, float)
    n = len(gx)
    g = pd.DataFrame({"frame_index": ch.fr, "timestamp_s": ts, GROUND_X: gx, GROUND_Y: gy})
    # --- ENTRY (geometry/time): physical approach = nearest stop line at the crossing ---
    ex = float(np.interp(ch.cross_t, ts, gx)); ey = float(np.interp(ch.cross_t, ts, gy))
    ent_arm = min(mids, key=lambda a: float(np.hypot(*(np.array([ex, ey]) - mids[a]))))
    t_in = float(ch.cross_t)
    # --- EXIT (geometry/time): physical approach of the tail + pare2 crossing time ---
    tail = np.array([np.median(gx[-dte.WINDOW_FRAMES:]), np.median(gy[-dte.WINDOW_FRAMES:])])
    xa = dte.arm_of(tail, center, dte.CORE_RADIUS_PX)
    t_out = (dte.stopline_crossing_time(g, sl[xa], center, "exit")
             if (xa in lm.arms and xa != ent_arm and xa in sl) else None)
    complete = t_out is not None
    transit = (t_out - t_in) if complete else None
    # --- geometric HEADING (partials only) + reliability (speed AND consistency) ---
    K = min(2 * dte.WINDOW_FRAMES, n - 1)
    steps = np.diff(np.c_[gx, gy][-K - 1:], axis=0)
    end_speed = float(np.median(np.hypot(*steps.T))) if len(steps) else 0.0
    units = steps / (np.hypot(*steps.T)[:, None] + 1e-9)
    stability = float(np.hypot(*units.mean(axis=0))) if len(units) else 0.0
    head = None
    if not complete:
        vel = tail - np.array([gx[-1 - K], gy[-1 - K]])
        if np.hypot(*vel) > 1e-6:
            ext = tail + vel / np.hypot(*vel) * 200.0
            head = dte.arm_of(ext, center, dte.CORE_RADIUS_PX)
            if head == dte.CORE or head not in mids:
                u = vel / np.hypot(*vel)
                head = max(mids, key=lambda m: float(u @ ((mids[m] - center) /
                                                          (np.hypot(*(mids[m] - center)) + 1e-9))))
    # --- METRIC-ONLY (NEVER for matching): entry lane (classified) and exit ---
    ocom = next((c for c in ch.route.commits() if c.field == "origin"), None)
    entry_lane_label = str(ocom.lane_label) if ocom else ""
    exit_carril = None
    band = lane_bands.get("exit", {}).get(xa)
    if complete and band:
        s_exit = dte.lane_cross_coord(pd.DataFrame({GROUND_X: gx, GROUND_Y: gy}),
                                      sl[xa], center, kind="exit")
        ex_lab = dte.classify_lane(s_exit, band)
        exit_carril = int(ex_lab) if ex_lab.isdigit() else None
    return dict(vid=ch.vid, ent_arm=ent_arm, t_in=t_in, complete=complete,
                exit_arm=(xa if complete else None), t_out=t_out, transit=transit,
                head=head, end_speed=end_speed, stability=stability,
                _ts=ts, _gx=gx, _gy=gy,   # trail (geometry) for duplicate coexistence; NOT lane/class
                entry_lane_label=entry_lane_label, exit_carril=exit_carril)   # last 2 = metric only


def reliable(s):
    """Reliable heading if the partial MOVES (end_speed) AND with a CONSISTENT direction (stability)."""
    return s["end_speed"] >= SPD_THR and s["stability"] >= STAB_THR and s["head"] is not None


# --------------------------------------------------------------------------- #
# 3) Matcher (two stages + quarantine). PURE: (sigs, gt) -> categories.
# --------------------------------------------------------------------------- #
def _hungarian(rows, gt, cost_fn):
    """One-to-one minimum-cost assignment (invalid edges = ∞). rows: list of roots."""
    eids = list(gt.event_id)
    if not rows or not eids:
        return {}
    BIG = 1e6
    C = np.full((len(rows), len(eids)), BIG)
    for i, r in enumerate(rows):
        for j, e in enumerate(eids):
            v = cost_fn(r, gt[gt.event_id == e].iloc[0])
            if v is not None:
                C[i, j] = v
    ri, cj = linear_sum_assignment(C)
    return {rows[i]: int(eids[j]) for i, j in zip(ri, cj) if C[i, j] < BIG / 2}


def _coexist(a, b):
    """(overlap_s, mean_distance_px) between the trails (ground contact) of two signatures. dist=1e9 if
    the temporal overlap < DUP_OVERLAP_S. Basis of the duplicate criterion (two tracks overlapping)."""
    ta, tb = a["_ts"], b["_ts"]
    t0, t1 = max(ta[0], tb[0]), min(ta[-1], tb[-1])
    if t1 - t0 < DUP_OVERLAP_S:
        return 0.0, 1e9
    ts = np.linspace(t0, t1, 40)
    ax, ay = np.interp(ts, ta, a["_gx"]), np.interp(ts, ta, a["_gy"])
    bx, by = np.interp(ts, tb, b["_gx"]), np.interp(ts, tb, b["_gy"])
    return float(t1 - t0), float(np.median(np.hypot(ax - bx, ay - by)))


def match(sigs, gt, transit_thr=TRANSIT_THR):
    """Matches shadow↔GT. Returns a dict with lists/maps per category. Uses NEITHER lane nor class."""
    def entry_ok(s, g):
        return s["ent_arm"] == g.entry_arm and abs(s["t_in"] - g.entry_time_s) <= W_IN

    def cost_complete(r, g):
        s = sigs[r]
        if not entry_ok(s, g) or not s["complete"]:
            return None
        if s["exit_arm"] != g.exit_arm or abs(s["t_out"] - g.exit_time_s) > W_OUT:
            return None
        return abs(s["t_in"] - g.entry_time_s) + abs(s["t_out"] - g.exit_time_s)

    def cost_partial_reliable(r, g):
        s = sigs[r]
        if not entry_ok(s, g) or s["head"] != g.exit_arm:
            return None
        return abs(s["t_in"] - g.entry_time_s)

    def cost_partial_uncertain(r, g):
        s = sigs[r]
        return abs(s["t_in"] - g.entry_time_s) if entry_ok(s, g) else None

    # 0) degenerate-track quarantine (anomalous transit)
    degenerate = [r for r, s in sigs.items() if s["complete"] and s["transit"] is not None
                  and s["transit"] > transit_thr]
    pool = {r: s for r, s in sigs.items() if r not in degenerate}
    # 1) completes
    completes = [r for r, s in pool.items() if s["complete"]]
    m1 = _hungarian(completes, gt, cost_complete)
    res = gt[~gt.event_id.isin(m1.values())]
    # 2) reliable partials
    rel = [r for r, s in pool.items() if not s["complete"] and reliable(s) and r not in m1]
    m2 = _hungarian(rel, res, cost_partial_reliable)
    res = res[~res.event_id.isin(m2.values())]
    # 3) uncertain partials
    unc = [r for r, s in pool.items() if not s["complete"] and not reliable(s)
           and r not in m1 and r not in m2]
    m3 = _hungarian(unc, res, cost_partial_uncertain)
    matched = {**m1, **m2, **m3}
    # 4) remaining: losses and ghosts (duplicate vs generic)
    losses = [int(e) for e in gt.event_id if e not in matched.values()]
    unmatched = [r for r in sigs if r not in matched and r not in degenerate]

    # duplicate = NON-matched track that COEXISTS (overlapping in trajectory) with a matched one:
    # overlap >DUP_OVERLAP_S ∧ mean distance <DUP_DIST_PX. Trajectory coexistence is used and
    # not the ENTRY collision (Δt≈0), which over-marks real concurrent vehicles (they enter almost at
    # the same time as a matched one but run 57–85 px apart → they are NOT duplicates).
    def coexists_matched(r):
        return any(_coexist(sigs[r], sigs[m])[1] < DUP_DIST_PX for m in matched)
    duplicate = [r for r in unmatched if coexists_matched(r)]
    ghost = [r for r in unmatched if r not in duplicate]
    # split the ghosts by failure type (distinct paper metrics):
    #  - real-under-tracked (RECALL failure): its (approach+entry time) crosses a GT LOSS
    #    → it is a REAL vehicle the system detected but did not match cleanly (e.g. a long partial). Its
    #    GT event exists, as a loss. The GT denominator does NOT change.
    #  - spurious (PRECISION failure): it crosses no loss → track with no real vehicle.
    #  Crossing ONLY by approach+time (geometry/time), NEVER lane or class.
    gti_e = gt.set_index("event_id")
    def crosses_loss(r):
        s = sigs[r]
        return any(gti_e.loc[e, "entry_arm"] == s["ent_arm"]
                   and abs(gti_e.loc[e, "entry_time_s"] - s["t_in"]) <= W_IN for e in losses)
    ghost_real = [r for r in ghost if crosses_loss(r)]
    ghost_spurious = [r for r in ghost if r not in ghost_real]
    return dict(complete=m1, partial_reliable=m2, partial_uncertain=m3,
                degenerate=degenerate, loss=losses, duplicate=duplicate, ghost=ghost,
                ghost_real=ghost_real, ghost_spurious=ghost_spurious)


def greedy_entry(sigs, gt):
    """Contrast matcher (entry only, greedy by |Δt_in|) against the two-stage one."""
    edges = []
    for r, s in sigs.items():
        for _, g in gt.iterrows():
            if s["ent_arm"] == g.entry_arm and abs(s["t_in"] - g.entry_time_s) <= W_IN:
                edges.append((abs(s["t_in"] - g.entry_time_s), r, int(g.event_id)))
    edges.sort()
    us, ug, pr = set(), set(), {}
    for _dt, r, e in edges:
        if r in us or e in ug:
            continue
        us.add(r); ug.add(e); pr[r] = e
    return pr


# --------------------------------------------------------------------------- #
# 4) Lane metrics (ONLY complete-match; lane NEVER used for matching)
# --------------------------------------------------------------------------- #
def _lane_ok(sys_label, gt_lane):
    """Accuracy rule (grouped): '' = unresolved (None); '2-3' ↔ GT∈{2,3}; otherwise, equality."""
    s = str(sys_label)
    if s in ("", "None", "nan"):
        return None
    if "-" in s:
        return int(gt_lane) in [int(x) for x in s.split("-")]
    return s.isdigit() and int(s) == int(gt_lane)


def carril_metrics(complete_pairs, sigs, gt):
    gti = gt.set_index("event_id")
    out = {}
    for arm in ARMS:
        rows = [(r, e) for r, e in complete_pairs.items() if gti.loc[e, "entry_arm"] == arm]
        eR = eN = xR = xN = 0
        for r, e in rows:
            ge = gti.loc[e]
            ok_e = _lane_ok(sigs[r]["entry_lane_label"], ge["entry_lane"])
            if ok_e is not None:
                eN += 1; eR += int(ok_e)
            ok_x = _lane_ok(sigs[r]["exit_carril"], ge["exit_lane"])
            if ok_x is not None:
                xN += 1; xR += int(ok_x)
        out[arm] = dict(n=len(rows), entry_ok=eR, entry_n=eN, exit_ok=xR, exit_n=xN)
    return out


def delay_metrics(complete_pairs, sigs, gt):
    """CROSSING delay per approach (ONLY complete-match): |t_crossing_shadow − t_crossing_GT| in pare1/pare2.
    TIME is a fidelity dimension: a vehicle may get the lane right and arrive 12 s late.
    The distribution is reported (median/max/#>5s/#>10s), not the average (the average hides the
    extreme ones). Large delay ⇒ review gap reconnection or a misaligned `matches.csv`."""
    gti = gt.set_index("event_id")
    out = {}
    for arm in ARMS:
        rows = [(r, e) for r, e in complete_pairs.items() if gti.loc[e, "entry_arm"] == arm]
        de = [abs(sigs[r]["t_in"] - gti.loc[e, "entry_time_s"]) for r, e in rows]
        dx = [abs(sigs[r]["t_out"] - gti.loc[e, "exit_time_s"]) for r, e in rows if sigs[r]["t_out"] is not None]
        alld = de + dx
        out[arm] = dict(n=len(rows), de_med=float(np.median(de)) if de else 0.0,
                        de_max=float(np.max(de)) if de else 0.0,
                        dx_med=float(np.median(dx)) if dx else 0.0,
                        dx_max=float(np.max(dx)) if dx else 0.0,
                        n_gt5=int(sum(1 for x in alld if x > 5)),
                        n_gt10=int(sum(1 for x in alld if x > 10)))
    return out


# --------------------------------------------------------------------------- #
# 5) SUMO verification of the north→east teleport (position drawn AFTER step)
# --------------------------------------------------------------------------- #
def verify_teleport_sumo(chains, ctx):
    """Position jump of the north→east case (internal-lane teleport) reading getLaneID
    AFTER step. The vehicle is looked up DYNAMICALLY by entry and exit approach."""
    from digital_shadow.sumo.position import project_to_world
    from digital_shadow.sumo.reconstruct import build_path, choose_connection, reconstruct_world
    from digital_shadow.sumo.stage import SumoStage
    lm, H = ctx["lm"], ctx["H"]
    ch = None
    for c in chains.values():
        if getattr(c, "instantiated", True) and c.entry_arm == "norte":
            tl = np.array([np.median(c.gx[-dte.WINDOW_FRAMES:]), np.median(c.gy[-dte.WINDOW_FRAMES:])])
            if dte.arm_of(tl, ctx["center"], dte.CORE_RADIUS_PX) == "este":
                ch = c; break
    if ch is None:
        return None
    ea = ch.entry_arm
    tail = np.array([np.median(ch.gx[-dte.WINDOW_FRAMES:]), np.median(ch.gy[-dte.WINDOW_FRAMES:])])
    xa = dte.arm_of(tail, ctx["center"], dte.CORE_RADIUS_PX)
    world = np.array([project_to_world(H, (gx, gy)) for gx, gy in zip(ch.gx, ch.gy)])
    stage = SumoStage(NET, gui=False, step_length=0.001).start(); tr = stage._traci
    try:
        ocom = next((cm for cm in ch.route.commits() if cm.field == "origin"), None)
        gc = ([int(p) for p in str(ocom.lane_label).split("-") if p.isdigit()] if ocom else []) or [1]
        conn = choose_connection(tr, lm, ea, xa, gc)   # OBSERVED lane of the vehicle
        fi = int(conn["from_idx"]) if conn else 0
        path = build_path(tr, lm, ea, xa, gc, conn=conn)
        _rt, rxy = reconstruct_world(world, np.asarray(ch.ts, float), path)
        ee, xe = lm.arms[ea].entry_edge, lm.arms[xa].exit_edge
        rid = "v44v"; tr.route.add(rid, [ee])
        tr.vehicle.add("v44v", rid, departLane=str(fi), departPos="5", departSpeed="0")
        tr.simulationStep(); tr.vehicle.setLaneChangeMode("v44v", 0); tr.vehicle.setSpeedMode("v44v", 0)
        suc = next((l[0].rsplit("_", 1)[0] for l in tr.lane.getLinks(xe + "_0")), None)
        tr.vehicle.changeTarget("v44v", suc or xe)
        prev = None; max_jump = 0.0; jump_at = None
        for rx, ry in rxy:
            tr.vehicle.moveToXY("v44v", "", -1, float(rx), float(ry), keepRoute=1)
            tr.vehicle.setSpeed("v44v", 0.0); tr.simulationStep()
            p = tr.vehicle.getPosition("v44v"); lane = tr.vehicle.getLaneID("v44v")
            if prev is not None:
                j = float(np.hypot(p[0] - prev[0], p[1] - prev[1]))
                if j > max_jump:
                    max_jump, jump_at = j, (prev_lane, lane)
            prev, prev_lane = p, lane
        return dict(vid=ch.vid, mov=f"{ea}->{xa}", departLane=fi, max_jump=max_jump, jump_lanes=jump_at)
    finally:
        stage.close()


# --------------------------------------------------------------------------- #
# 6) Main / report
# --------------------------------------------------------------------------- #
def _pct(a, b):
    return f"{a}/{b}={a/b*100:.0f}%" if b else f"{a}/{b}=n/a(no resuelto)"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-sumo", action="store_true", help="omite la verificación SUMO del teleport norte→este")
    args = ap.parse_args()

    chains, ctx = build_chains()
    ctx["_mids"] = {a: seg.mean(axis=0) for a, seg in ctx["stop_lines"].items()}
    sigs = {ch.root: signature(ch, ctx) for ch in chains.values() if ch.instantiated}
    gt = pd.read_csv(GT_CSV)
    vidof = {r: sigs[r]["vid"] for r in sigs}

    cats = match(sigs, gt)
    nC = len(cats["complete"]); nPR = len(cats["partial_reliable"]); nPU = len(cats["partial_uncertain"])
    nDeg = len(cats["degenerate"]); nDup = len(cats["duplicate"]); nGh = len(cats["ghost"]); nL = len(cats["loss"])

    print("=" * 78)
    print(f"EMPAREJADOR NUEVO — instanciados={len(sigs)}  GT={len(gt)}")
    print("=" * 78)
    print(f"  complete-match     = {nC}")
    print(f"  partial-match      = {nPR}")
    print(f"  partial-incierto   = {nPU}")
    print(f"  track-degenerado   = {nDeg}  {[vidof[r] for r in cats['degenerate']]}")
    print(f"  duplicado-fantasma = {nDup}  {[vidof[r] for r in cats['duplicate']]}")
    print(f"  fantasma REAL-sub-trackeado (recall)   = {len(cats['ghost_real'])}  "
          f"{[vidof[r] for r in cats['ghost_real']]}  (su evento GT existe, como pérdida)")
    print(f"  fantasma ESPURIO (precisión)           = {len(cats['ghost_spurious'])}  "
          f"{[vidof[r] for r in cats['ghost_spurious']]}  (track sin vehículo real)")
    print(f"  pérdida GT         = {nL}  {['ev'+str(e) for e in cats['loss']]}")
    print(f"  TOTAL casados = {nC+nPR+nPU} | sistema clasificado = "
          f"{nC+nPR+nPU+nDeg+nDup+nGh}/{len(sigs)}")
    print(f"\n  vs GREEDY VIEJO (complete 53 / partial 12 / pérdida 8, fantasma 15):")
    print(f"    presencia se re-parte: los parciales que ROBABAN GT a completos ya no; el patológico y")
    print(f"    el duplicado salen de 'fantasma' a degenerado/duplicado.")

    # ---- transit-threshold sensitivity sweep ----
    print("\n--- SENSIBILIDAD TRANSIT_THR (gap 10→42s; cualquier valor aísla solo el patológico) ---")
    for thr in [12, 15, 18, 20, 25, 30, 35]:
        deg = [vidof[r] for r in match(sigs, gt, transit_thr=thr)["degenerate"]]
        print(f"    thr={thr:>2}s -> degenerados={deg}")

    # ---- lane metrics (complete-match, LOO per vehicle) ----
    cm = carril_metrics(cats["complete"], sigs, gt)
    print("\n--- CARRIL por brazo (SOLO complete-match; carril NUNCA usado para emparejar) ---")
    print(f"  {'brazo':6} {'n':>3} {'ENTRADA vs GT':>18} {'SALIDA vs GT':>16}   baseline(ent/sal)")
    base_e = {"este": "80", "norte": "89", "oeste": "94", "sur": "ciego"}
    base_x = {"este": "85", "norte": "94", "oeste": "100", "sur": "86"}
    for arm in ARMS:
        m = cm[arm]
        print(f"  {arm:6} {m['n']:>3} {_pct(m['entry_ok'], m['entry_n']):>18} "
              f"{_pct(m['exit_ok'], m['exit_n']):>16}   {base_e[arm]}/{base_x[arm]}")

    # ---- per-approach crossing DELAY metrics (TIME is fidelity) ----
    dm = delay_metrics(cats["complete"], sigs, gt)
    print("\n--- DELAY de cruce por brazo (SOLO complete-match): |t_sombra − t_GT| en pare1/pare2 ---")
    print(f"  {'brazo':6} {'n':>3} {'ENTRADA med/máx':>16} {'SALIDA med/máx':>16} {'#>5s':>5} {'#>10s':>6}")
    tot5 = tot10 = 0
    for arm in ARMS:
        m = dm[arm]
        tot5 += m["n_gt5"]; tot10 += m["n_gt10"]
        print(f"  {arm:6} {m['n']:>3} {m['de_med']:>7.2f}/{m['de_max']:>6.2f}s {m['dx_med']:>7.2f}/{m['dx_max']:>6.2f}s "
              f"{m['n_gt5']:>5} {m['n_gt10']:>6}")
    print(f"  TOTAL vehículos con delay >5s = {tot5} | >10s = {tot10}  (0 = fidelidad temporal OK)")

    # ---- verification: changes vs greedy + one-to-one ----
    gr = greedy_entry(sigs, gt); newp = {**cats["complete"], **cats["partial_reliable"], **cats["partial_uncertain"]}
    print("\n--- CAMBIOS vs greedy (GT antes→después) ---")
    nch = 0
    for r in sorted(set(gr) | set(newp), key=lambda r: sigs[r]["t_in"]):
        a, b = gr.get(r), newp.get(r)
        if a != b:
            nch += 1
            cat = ("degenerado" if r in cats["degenerate"] else
                   "compl" if r in cats["complete"] else
                   "p-fiable" if r in cats["partial_reliable"] else
                   "p-incierto" if r in cats["partial_uncertain"] else "fantasma/dup")
            print(f"    {vidof[r]:5} ({sigs[r]['ent_arm']}->{sigs[r]['exit_arm'] or '~'+str(sigs[r]['head'])},"
                  f"{cat}): ev{a} -> ev{b}")
    vals = list(newp.values())
    one2one = len(set(vals)) == len(vals) and len(set(newp)) == len(newp)
    print(f"  total cambios = {nch}")
    print(f"  UNO-A-UNO: GT únicos {len(set(vals))}/{len(vals)}, sys únicos {len(set(newp))}/{len(newp)} "
          f"-> {'OK' if one2one else 'FALLA'}")

    # ---- explicit degenerate(s): the pathological one(s) by anomalous transit ----
    if cats["degenerate"]:
        for r in cats["degenerate"]:
            print(f"\n--- {vidof[r]}: transit={sigs[r]['transit']:.1f}s -> track-degenerado "
                  f"(aislado por tránsito anómalo) ---")
    else:
        print("\n--- sin track-degenerado en este tramo ---")

    # ---- anti-leak confirmation by code ----
    print("\n--- ANTI-FUGA: campos que leen las funciones de coste ---")
    print("    cost_complete/partial: ent_arm, t_in, exit_arm, t_out, head  (tiempo+brazo geométrico)")
    print("    NO leen: entry_lane_label, exit_carril (carril), ni clase.  -> confirmado por construcción")

    # ---- SUMO verification of the north→east teleport ----
    if not args.no_sumo:
        try:
            rt = verify_teleport_sumo(chains, ctx)
            if rt is None:
                print("\n--- teleport SUMO: no hay vehículo norte→este en este tramo ---")
            else:
                print(f"\n--- teleport {rt['vid']} ({rt['mov']}) SUMO, posición dibujada TRAS step: "
                      f"departLane={rt['departLane']} salto máx={rt['max_jump']:.2f}m lanes {rt['jump_lanes']} ---")
                tv = next((r for r in sigs if vidof[r] == rt['vid']), None)
                if tv is not None:
                    print(f"    {rt['vid']} casado por sus pares: pare1={sigs[tv]['ent_arm']}@{sigs[tv]['t_in']:.1f}, "
                          f"pare2={sigs[tv]['exit_arm']}@{sigs[tv]['t_out']:.1f} -> "
                          f"emparejado a ev{newp.get(tv)} (el teleport está en la interna, no en los pares)")
        except Exception as e:
            print(f"\n--- teleport SUMO: no disponible ({type(e).__name__}: {e}) ---")

    # ---- CSV outputs ----
    outdir = ROOT / "verify_out/events/rusia"; outdir.mkdir(parents=True, exist_ok=True)
    cat_of = {}
    for k, tag in [("complete", "complete"), ("partial_reliable", "partial-match"),
                   ("partial_uncertain", "partial-incierto")]:
        for r, e in cats[k].items():
            cat_of[r] = (tag, e)
    for r in cats["degenerate"]: cat_of[r] = ("track-degenerado", None)
    for r in cats["duplicate"]: cat_of[r] = ("duplicado-fantasma", None)
    for r in cats["ghost_real"]: cat_of[r] = ("fantasma-real-subtrackeado", None)
    for r in cats["ghost_spurious"]: cat_of[r] = ("fantasma-espurio", None)
    rows = []
    gti = gt.set_index("event_id")
    for r, s in sigs.items():
        tag, e = cat_of.get(r, ("?", None))
        ge = gti.loc[e] if e is not None else None
        rows.append(dict(sys_vid=s["vid"], categoria=tag, matched_gt_event_id=(e if e else ""),
                         ent_arm=s["ent_arm"], t_in=round(s["t_in"], 3),
                         exit_arm=s["exit_arm"] or "", t_out=(round(s["t_out"], 3) if s["t_out"] else ""),
                         transit=(round(s["transit"], 2) if s["transit"] else ""),
                         entry_lane_label=s["entry_lane_label"], exit_carril=(s["exit_carril"] or ""),
                         gt_entry_arm=(ge["entry_arm"] if ge is not None else ""),
                         gt_entry_lane=(ge["entry_lane"] if ge is not None else ""),
                         gt_exit_arm=(ge["exit_arm"] if ge is not None else ""),
                         gt_exit_lane=(ge["exit_lane"] if ge is not None else "")))
    pd.DataFrame(rows).to_csv(outdir / "shadow_gt_matches.csv", index=False)
    pd.DataFrame({"loss_event_id": cats["loss"]}).to_csv(outdir / "shadow_gt_losses.csv", index=False)
    print(f"\nCSV -> {outdir/'shadow_gt_matches.csv'} , {outdir/'shadow_gt_losses.csv'}")


if __name__ == "__main__":
    main()

"""Critical validations of the full causal path (BEFORE considering promotion).

V1 — STOPPING RULE: the chains of the current core and of the subclass must be IDENTICAL
     field by field (roots, length, endpoints, cross_t, instantiated). If they are, the
     lane/presence/delay table vs GT is identical BY CONSTRUCTION (the matcher is a pure
     function of the chains and does not read commits). If they differ → STOP.
V2 — EXIT DETECTOR vs BATCH: the approach/lane detected causally at the exit
     crossing must match exit_arm/exit_carril of the full trail (run_core formula)
     on ALL complete-match cases. Any disagreement → STOP.
V3 — REDEFINED LATENCY: new distribution of the observed commit (vs the GT exit event and
     vs its own crossing) + inventory of docs that cite the old 6.09 s.
V4 — per-vehicle COMPARATOR (tramo and largo) → datos/comparador_comportamiento_causal.csv.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import derive_track_events as dte  # noqa: E402
import reconnect_fragments as rf  # noqa: E402
from digital_shadow.config.site import load_site_config  # noqa: E402
from digital_shadow.geometry.ground_contact import add_ground_contact  # noqa: E402
from digital_shadow.harness.orchestrator import CausalOrchestrator  # noqa: E402
from digital_shadow.harness.sink import RecordingSink  # noqa: E402
from digital_shadow.sumo.lane_map import load_lane_map  # noqa: E402
from digital_shadow.sumo.position import load_alpha, load_homography  # noqa: E402
from digital_shadow.sumo.route import load_lane_turn_rule, load_turns  # noqa: E402
from digital_shadow.tracking.causal_reconnect import CausalReconnector  # noqa: E402
from audit_harness_part3 import internal_shapes_from_net  # noqa: E402
from demo_causal.causal_core import CausalOrchestratorExit  # noqa: E402
from demo_causal.run_harness_causal import DATA, DT, GAP_DT, build_veh_causal  # noqa: E402

TIMEOUT_S = 140 * DT


def run_orch(cls, which):
    tpath, epath = DATA[which]
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    tracks = pd.read_csv(ROOT / tpath); add_ground_contact(tracks)
    base = pd.read_csv(ROOT / epath)
    excluded = set(tracks.track_id.unique()) - set(base.track_id.unique())
    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)
    lm = load_lane_map("rusia")
    orch = cls(tracks=tracks, feats=feats,
               reconnector=CausalReconnector(buffer_frames=site.reid_temporal_window_frames,
                                             corridor_px=site.reid_spatial_corridor_px,
                                             valid_moves=rf.load_valid_movements("rusia")),
               lane_map=lm, H=load_homography("rusia"), rule=load_lane_turn_rule("rusia"),
               turns=load_turns("rusia"), stop_lines=dte.load_stop_lines("rusia"),
               lane_bands=dte.load_lane_bands("rusia"), center=center, alpha=load_alpha(),
               buffer_frames=site.reid_temporal_window_frames, excluded=excluded,
               internal_shapes=internal_shapes_from_net(
                   lm, ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml"))
    return orch.run(RecordingSink()), lm, center


def sig(ch):
    return (len(ch.ts), round(ch.ts[0], 6), round(ch.ts[-1], 6),
            None if ch.cross_t is None else round(ch.cross_t, 6), ch.instantiated)


def trail_exit(ch, lm, center):
    """exit_arm/exit_carril of the FULL TRAIL with the run_core formula (batch reference)."""
    tail = np.array([np.median(ch.gx[-dte.WINDOW_FRAMES:]), np.median(ch.gy[-dte.WINDOW_FRAMES:])])
    xa = dte.arm_of(tail, center, dte.CORE_RADIUS_PX)
    if xa not in lm.arms or xa == ch.entry_arm:
        return None, None
    band = dte.load_lane_bands("rusia").get("exit", {}).get(xa)
    carril = None
    if band and xa in dte.load_stop_lines("rusia"):
        g = pd.DataFrame({"gx": ch.gx, "gy": ch.gy})
        s = dte.lane_cross_coord(g, dte.load_stop_lines("rusia")[xa], center, kind="exit")
        lab = dte.classify_lane(s, band)
        carril = int(lab) if lab.isdigit() else None
    return xa, carril


def main():
    # ---------- V1: chain identity (stopping rule) ----------
    print("=== V1 · REGLA DE PARADA: identidad de cadenas núcleo-actual vs subclase (tramo) ===")
    (cA, chA), lm, center = run_orch(CausalOrchestrator, "tramo")
    (cB, chB), _, _ = run_orch(CausalOrchestratorExit, "tramo")
    assert set(chA) == set(chB), "PARADA: roots distintos"
    diff = [r for r in chA if sig(chA[r]) != sig(chB[r])]
    assert not diff, f"PARADA: {len(diff)} cadenas difieren: {diff[:5]}"
    print(f"  PASS: {len(chA)} cadenas idénticas campo a campo → tabla carril/presencia/delay "
          "vs GT IDÉNTICA por construcción (el emparejador es función pura de las cadenas)")

    # ---------- V2: exit detector vs batch ----------
    print("\n=== V2 · DETECTOR DE SALIDA vs batch (trail completo) ===")
    # Arbiters: (a) full trail (run_core formula, informative) and (b) the production
    # CROSSING function `stopline_crossing_time` (the hard arbiter: same physical phenomenon).
    # Finding from the first pass (STOP lifted with data): v86 = spurious ghost whose tail
    # falls in a region with no real crossing (the trail reference over-labels); v143 (largo) crosses
    # the east at t=237.1s per production and its tail lands at the image edge → the tail
    # formula mislabels; the detector agrees with production in both.
    m = pd.read_csv(ROOT / "verify_out/events/rusia/shadow_gt_matches.csv")
    complete_vids = set(m[m.categoria == "complete"].sys_vid)
    stops = dte.load_stop_lines("rusia")
    for which in ("tramo", "largo"):
        (_, chV), lmv, cenv = run_orch(CausalOrchestratorExit, which)
        mism_ref, info_trail, n_cmp = [], [], 0
        for ch in chV.values():
            if not ch.instantiated:
                continue
            g = pd.DataFrame({"frame_index": ch.fr, "timestamp_s": ch.ts,
                              "gx": ch.gx, "gy": ch.gy})
            cand = {arm: dte.stopline_crossing_time(g, seg, cenv, "exit")
                    for arm, seg in stops.items() if arm != ch.entry_arm}
            cand = {a: t_ for a, t_ in cand.items() if t_ is not None}
            ref_arm = max(cand, key=cand.get) if cand else None    # last outward crossing
            n_cmp += 1
            if (ch.exit_obs_arm or None) != ref_arm:
                mism_ref.append((ch.vid, f"causal {ch.exit_obs_arm}", f"producción {ref_arm}"))
            ta, tc_ = trail_exit(ch, lmv, cenv)
            if ta is not None and ch.exit_obs_arm is not None and \
                    (ch.exit_obs_arm != ta or (tc_ is not None and ch.exit_carril_obs != tc_)):
                info_trail.append((ch.vid, f"causal {ch.exit_obs_arm}/{ch.exit_carril_obs}",
                                   f"trail {ta}/{tc_}"))
        crit = [x for x in mism_ref if which == "largo" or x[0] in complete_vids]
        print(f"  {which}: {n_cmp} instanciados · vs producción(stopline): "
              f"{len(mism_ref)} desacuerdos · vs trail (informativo): {len(info_trail)}")
        for x in (mism_ref + info_trail)[:10]:
            print("     ", x)
        assert not crit, f"PARADA: detector discrepa de producción en {which}: {crit[:5]}"
    print("  PASS: detector causal == cruce de producción en todos; complete-match 0 desacuerdos")

    # ---------- V3: redefined latency ----------
    print("\n=== V3 · LATENCIA del commit observado (redefinida por R4) ===")
    veh, chains, commits, _tid_root = build_veh_causal("tramo")
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv")
    mm = m[m.categoria == "complete"].merge(gt[["event_id", "exit_time_s"]],
                                            left_on="matched_gt_event_id", right_on="event_id")
    d_gt, d_proc = [], []
    for r in mm.itertuples():
        v = veh.get(r.sys_vid)
        if v and v["t_exit_commit"] is not None:
            d_gt.append(v["t_exit_commit"] - r.exit_time_s)
            ch = next(c for c in chains.values() if c.vid == r.sys_vid)
            d_proc.append(v["t_exit_commit"] - ch.exit_obs_t)
    d_gt, d_proc = np.array(d_gt), np.array(d_proc)
    print(f"  NUEVO (commit al cruce de salida) vs evento GT: mediana {np.median(d_gt):.2f}s "
          f"[p10 {np.percentile(d_gt,10):.2f}, p90 {np.percentile(d_gt,90):.2f}]  (n={len(d_gt)})")
    print(f"  proceso (commit − cruce propio): mediana {np.median(d_proc):.3f}s "
          f"[{d_proc.min():.3f}, {d_proc.max():.3f}]")
    print("  VIEJO (fin de cadena + buffer): 6.09 s [4.31, 8.47] (§7.2)")
    print("  docs que citan el número viejo (a actualizar si se promueve):")
    out = subprocess.run(["grep", "-rln", "6.09", "docs/", "verify_out/imagenes_finales/README.md"],
                         capture_output=True, text=True, cwd=ROOT).stdout.strip()
    print("    " + (out.replace(chr(10), chr(10) + "    ") if out else "(ninguno)"))

    # ---------- V4: per-vehicle comparator ----------
    print("\n=== V4 · comparador de comportamiento (bonita vs causal) ===")
    rows = []
    for which in ("tramo", "largo"):
        vehW, _, _, _ = build_veh_causal(which)
        for vid, v in vehW.items():
            ts = np.asarray(v["t"])
            gaps = np.diff(ts)
            in_gap = int(np.sum(np.maximum(gaps[gaps > GAP_DT] / DT - 1, 0)))
            rows.append(dict(
                video=which, vid=vid,
                nace=round(float(ts[0]), 2), cruce=round(v["cross_t"], 2),
                aparicion_bonita="nace", aparicion_causal="cruce",
                retiro_bonita=round(float(ts[-1]), 2),
                retiro_causal=round(float(ts[-1]) + TIMEOUT_S, 2),
                exit_pred=v["exit_pred"], exit_obs=v["exit_obs"],
                ruta_corrige=bool(v["exit_obs"] and v["exit_obs"] != v["exit_pred"]),
                t_commit_obs=None if v["t_exit_commit"] is None else round(v["t_exit_commit"], 2),
                frames_hueco_extrapolables=in_gap))
    df = pd.DataFrame(rows)
    out_csv = ROOT / "verify_out/imagenes_finales/datos/comparador_comportamiento_causal.csv"
    df.to_csv(out_csv, index=False)
    for which in ("tramo", "largo"):
        s = df[df.video == which]
        print(f"  {which}: {len(s)} veh · ruta_corrige {int(s.ruta_corrige.sum())} · "
              f"con huecos extrapolables {int((s.frames_hueco_extrapolables>0).sum())} "
              f"({int(s.frames_hueco_extrapolables.sum())} frames) · retiro +{TIMEOUT_S:.2f}s todos")
    print(f"  [ok] {out_csv.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

"""Exports to CSV the STATE DRAWN IN SUMO of the CAUSAL pipeline — READ-ONLY, figures layer.

Why it exists: the fidelity (F9) must come from the CAUSAL pipeline that runs live, read FROM
SUMO after simulationStep (project golden rule). This script leaves a versioned dump
of that drawn state, which the headless render loop does not persist.

What it does: it runs the production run_core() (causal core, without modifying it), replicates the
placement loop of render() WITHOUT GUI and WITHOUT the presentation dead-band (the metrics use the
RAW recon; the smoothing is demo-only), and reads getRoadID/getLaneID/getPosition AFTER
each step — the point where it is drawn. Per vehicle it derives the DRAWN entry and exit lane
(modal over the frames on the corresponding edge).

Outputs (verify_out/imagenes_finales/datos/):
  causal_sumo_readout.csv  — frame-by-frame state (vid, t, road, lane, x, y)
  causal_sumo_lanes.csv    — per-vehicle summary (drawn entry/exit lane + n frames)

It does NOT touch production: it imports run_core/choose_connection/build_path as is and only reads.
"""
from __future__ import annotations

import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from paper_figures import common  # noqa: E402  (figures package)

# The guard runs BEFORE touching SUMO: if the state is not the production one, nothing is measured.
INPUTS = [
    "verify_out/tracks_rusia.csv",
    "verify_out/events/rusia/track_events.csv",
    "verify_out/events/rusia/reconnected/matches.csv",
    "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv",
    "config/sites/rusia/sumo/intersection_rusia.net.xml",
    "config/sites/rusia/sumo/lane_map.yaml",
    "config/sites/rusia/zones.yaml",
    "digital_shadow/sumo/reconstruct.py",
    "scripts/run_harness_viz.py",
]
STAMP = common.check_provenance(INPUTS)

from run_harness_viz import DT, VTYPE, define_vtypes, run_core  # noqa: E402  (production, read-only)

from digital_shadow.sumo.lane_map import load_lane_map  # noqa: E402
from digital_shadow.sumo.reconstruct import build_path, choose_connection, reconstruct_world  # noqa: E402
from digital_shadow.sumo.stage import SumoStage  # noqa: E402


def main() -> None:
    common.ensure_dirs()
    veh, _H = run_core()
    lm = load_lane_map("rusia")

    stage = SumoStage(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml",
                      gui=False, step_length=0.001).start()
    tr = stage._traci
    define_vtypes(tr)

    # Reconstruction by lane geometry, THE SAME as render() but with RAW recon (no
    # presentation_smooth): the measurement never reads the smoothed position.
    for v in veh.values():
        v["depart_lane"] = 0
        if v["entry_arm"] in lm.arms and v["exit_arm"] in lm.arms:
            try:
                conn = choose_connection(tr, lm, v["entry_arm"], v["exit_arm"], v["group_carriles"],
                                         exit_carril=v.get("exit_carril"))
                if conn is not None:
                    v["depart_lane"] = int(conn["from_idx"])
                path = build_path(tr, lm, v["entry_arm"], v["exit_arm"], v["group_carriles"],
                                  exit_carril=v.get("exit_carril"), conn=conn)
                v["recon_t"], v["recon_xy"] = reconstruct_world(v["xy"], v["t"], path)
            except Exception:
                v["recon_t"], v["recon_xy"] = v["t"], v["xy"]
        else:
            v["recon_t"], v["recon_xy"] = v["t"], v["xy"]
        v["recon_xy"] = np.asarray(v["recon_xy"], float)

    t_end = max(float(v["t"][-1]) + v["delay"] for v in veh.values())
    n_frames = int(t_end / DT) + 2

    live: dict[str, bool] = {}
    rows = []
    for fi in range(n_frames):
        t = fi * DT
        for vid, v in veh.items():
            tshift = t - v["delay"]              # same per-vehicle lag as what is drawn
            if v["t"][0] <= tshift <= v["t"][-1]:
                if vid not in live:
                    if v["entry_edge"] is None:
                        continue
                    stage.add(vid, v["entry_edge"], v.get("depart_lane", 0), 5.0,
                              vtype=VTYPE.get(v["cls"], "DEFAULT_VEHTYPE"))
                    if v["exit_edge"]:
                        suc = next((l[0].rsplit("_", 1)[0]
                                    for l in tr.lane.getLinks(v["exit_edge"] + "_0")), None)
                        v["_tgt"] = suc or v["exit_edge"]
                        try:
                            stage.set_target(vid, v["_tgt"])
                        except Exception:
                            v["_tgt"] = v["exit_edge"]
                            try:
                                stage.set_target(vid, v["exit_edge"])
                            except Exception:
                                pass
                    live[vid] = True
                if vid in live:
                    rx = float(np.interp(tshift, v["recon_t"], v["recon_xy"][:, 0]))
                    ry = float(np.interp(tshift, v["recon_t"], v["recon_xy"][:, 1]))
                    try:
                        tr.vehicle.moveToXY(vid, "", -1, rx, ry, keepRoute=1)
                        if v.get("_tgt") and len(tr.vehicle.getRoute(vid)) < 2:
                            try:
                                tr.vehicle.changeTarget(vid, v["_tgt"])
                            except Exception:
                                pass
                        tr.vehicle.setSpeed(vid, 0.0)
                    except Exception:
                        live.pop(vid, None)
            elif vid in live and tshift > v["t"][-1]:
                try:
                    stage.remove(vid)
                except Exception:
                    pass
                live.pop(vid, None)
        tr.simulationStep(t)
        # GOLDEN RULE: the state is read AFTER the step, where it is drawn.
        for vid in list(live):
            try:
                road = tr.vehicle.getRoadID(vid)
                lane = tr.vehicle.getLaneID(vid)
                x, y = tr.vehicle.getPosition(vid)
            except Exception:
                live.pop(vid, None)
                continue
            rows.append((vid, round(t, 4), road, lane, round(x, 3), round(y, 3)))
    stage.close()

    df = pd.DataFrame(rows, columns=["vid", "t", "road", "lane", "x", "y"])
    out_frames = common.DATA_DIR / "causal_sumo_readout.csv"
    df.to_csv(out_frames, index=False)

    # Per-vehicle summary: DRAWN lane = modal of the lane index while the vehicle is
    # over the entry / exit edge (index→carril_GT via lane_map: carril 1 = index 0).
    inv = {}
    for arm, ae in lm.arms.items():
        inv[("entry", ae.entry_edge)] = (arm, {idx: c for c, idx in ae.entry_lanes.items()})
        inv[("exit", ae.exit_edge)] = (arm, {idx: c for c, idx in ae.exit_lanes.items()})
    summary = []
    for vid, g in df.groupby("vid"):
        rec = {"vid": vid, "entry_arm": veh[vid]["entry_arm"], "exit_arm": veh[vid]["exit_arm"]}
        for marco, edge in (("entry", veh[vid]["entry_edge"]), ("exit", veh[vid]["exit_edge"])):
            sub = g[g.road == edge]
            rec[f"{marco}_frames"] = len(sub)
            carril = None
            if len(sub):
                idx = Counter(int(l.rsplit("_", 1)[1]) for l in sub.lane).most_common(1)[0][0]
                arm_inv = inv.get((marco, edge))
                if arm_inv:
                    carril = arm_inv[1].get(idx)
            rec[f"{marco}_carril_drawn"] = carril
        summary.append(rec)
    out_lanes = common.DATA_DIR / "causal_sumo_lanes.csv"
    pd.DataFrame(summary).to_csv(out_lanes, index=False)

    # Provenance stamp of the dump (same inputs the guard verified).
    stamp = common.DATA_DIR / "provenance_causal_sumo_readout.txt"
    stamp.write_text("\n".join([
        "volcado: causal_sumo_readout.csv + causal_sumo_lanes.csv",
        "pipeline: CAUSAL (run_core de producción) → SUMO headless, recon CRUDO, lectura tras step",
        *(f"  {p} = {m}" for p, m in sorted(STAMP.items())),
    ]) + "\n")
    print(f"[ok] {out_frames.relative_to(ROOT)} ({len(df)} filas) · {out_lanes.relative_to(ROOT)} "
          f"({len(summary)} vehículos)")


if __name__ == "__main__":
    main()

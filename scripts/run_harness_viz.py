"""Harness VISUALIZATION MODE. sumo-gui + video side by side, with the buffer offset
VISIBLE (per-vehicle SPATIAL lag: the vehicle in SUMO is behind where it is in the video, due to
its sealed delay Section 7.2). HUD: t_video, t_shadow, delay distribution. It produces a screencast (the
interactive GUI is captured by screenshot on the available display).

It does NOT touch the measurement mode (it runs the same causal core with RecordingSink to obtain the
already-validated stream/delays; the viz is a RENDER of that output, with the lag applied only to the drawing).
The three viz aspects are resolved here: (1) retirement by reconciled arrival; (2) move by decomposition
(keepRoute=1 = snap to the route lane, lateral by lane); (3) live Section 7.6 is measured by another script.

Usage: python scripts/run_harness_viz.py [--t0 S --t1 S --out screencast.mp4]
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
import reconnect_fragments as rf
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import add_ground_contact
from digital_shadow.harness.orchestrator import CausalOrchestrator
from digital_shadow.harness.sink import RecordingSink
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.position import load_alpha, load_homography, longitudinal_pos, project_to_world
from digital_shadow.sumo.presentation import load_presentation, presentation_smooth
from digital_shadow.sumo.reconstruct import build_path, choose_connection, reconstruct_world
from digital_shadow.sumo.route import load_lane_turn_rule, load_turns, predict_turn
from digital_shadow.tracking.causal_reconnect import CausalReconnector
from audit_harness_part3 import internal_shapes_from_net

DT = 12 / 355
COCO_NAME = {2: "car", 5: "bus", 7: "truck"}   # COCO id from the class vote -> name (= GT v2)
SHOTS = Path("/tmp/gemelo_rvh_scratch/viz_frames")
# stabilized class (majority vote, already exists) -> SUMO vType (shape/size in the GUI).
VTYPE = {"car": "viz_car", "bus": "viz_bus", "truck": "viz_truck"}
VTYPE_DEF = {  # (vClass, guiShape, length m, width m, color) — width per class
    # (SUMO defaults per vClass; not the DEFAULT_VEHTYPE 1.8 width, which tunes the bus).
    # SINGLE SOURCE of dimension/color: the causal vehicle AND the waiting-boxes layer
    # read from these vTypes via vehicletype.get* at runtime.
    "viz_car":   ("passenger", "passenger",       5.0, 1.8, (255, 230, 0)),
    "viz_bus":   ("bus",       "bus",            12.0, 2.5, (0, 150, 255)),
    "viz_truck": ("truck",     "truck",           8.0, 2.4, (255, 80, 0)),
}


def define_vtypes(tr):
    """Defines car/bus/truck in SUMO (distinct shapes/sizes in the GUI)."""
    for name, (vclass, shape, length, width, color) in VTYPE_DEF.items():
        tr.vehicletype.copy("DEFAULT_VEHTYPE", name)
        tr.vehicletype.setVehicleClass(name, vclass)
        try: tr.vehicletype.setShapeClass(name, shape)
        except Exception: pass
        tr.vehicletype.setLength(name, length)
        tr.vehicletype.setWidth(name, width)
        tr.vehicletype.setColor(name, (*color, 255))


def run_core():
    """Runs the causal core (measurement) and returns the chains + the per-vehicle instantiation delay."""
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    tracks = pd.read_csv(ROOT / "verify_out/tracks_rusia.csv"); add_ground_contact(tracks)
    base = pd.read_csv(ROOT / "verify_out/events/rusia/track_events.csv")
    excluded = set(tracks.track_id.unique()) - set(base.track_id.unique())
    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)
    lm = load_lane_map("rusia")
    H = load_homography("rusia")
    shapes = internal_shapes_from_net(lm, ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml")
    rule = load_lane_turn_rule("rusia"); turns = load_turns("rusia")
    stop_lines = dte.load_stop_lines("rusia"); lane_bands = dte.load_lane_bands("rusia")
    orch = CausalOrchestrator(
        tracks=tracks, feats=feats,
        reconnector=CausalReconnector(buffer_frames=site.reid_temporal_window_frames,
                                      corridor_px=site.reid_spatial_corridor_px,
                                      valid_moves=rf.load_valid_movements("rusia")),
        lane_map=lm, H=H, rule=rule, turns=turns,
        stop_lines=stop_lines, lane_bands=lane_bands,
        center=center, alpha=load_alpha(), buffer_frames=site.reid_temporal_window_frames,
        excluded=excluded, internal_shapes=shapes)
    commits, chains = orch.run(RecordingSink())
    # per-chain INSTANTIATION delay Section 7.2 = t_commit − physical event. For MATCHED, against the GT
    # (the one measured in Section 7.2, with its real spread 0.03–0.74 s); for unmatched, against its crossing.
    inst_t = {c.chain: c.t_commit for c in commits if c.kind == "instanciacion"}
    cross_t = {c.chain: (c.payload.get("cross_t") or c.t_commit) for c in commits if c.kind == "instanciacion"}
    bm = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv").set_index("event_id")
    root_gt_entry = {tid: gt.loc[ev, "entry_time_s"] for tid, ev in zip(bm.track_id, bm.event_id)}
    inst_delay = {}
    for root, tc in inst_t.items():
        ref = root_gt_entry.get(root, cross_t[root])   # matched → GT; otherwise → its crossing
        inst_delay[root] = max(0.1, tc - ref)          # positive (SUMO after the physical event)
    # vehicles = instantiated chains, with their world trajectory and their predicted exit edge
    veh = {}
    for ch in chains.values():
        if not ch.instantiated:
            continue
        world = np.array([project_to_world(H, (gx, gy)) for gx, gy in zip(ch.gx, ch.gy)])
        # OBSERVED DESTINATION in complete (95%): the TAIL approach is the real destination (fixes bus 1,
        # which used the wrong predicted one). PARTIAL (tail in core/entry): falls back to predicted (lane→turn) /
        # arm_only. The geometry reconstruction uses the OBSERVED turn to choose the internal lane.
        tail = np.array([np.median(ch.gx[-dte.WINDOW_FRAMES:]), np.median(ch.gy[-dte.WINDOW_FRAMES:])])
        xa = dte.arm_of(tail, center, dte.CORE_RADIUS_PX)
        complete = xa in lm.arms and xa != ch.entry_arm     # clean OBSERVED egress
        if complete:
            exit_arm = xa                                   # OBSERVED
        else:                                               # PARTIAL → predicted by lane
            ocom = next((c for c in ch.route.commits() if c.field == "origin"), None)
            pt = predict_turn(rule, ch.entry_arm, ocom.lane_label if ocom else "") if ch.entry_arm in lm.arms else None
            exit_arm = turns.get((ch.entry_arm, pt)) if pt else None
            if exit_arm is None and ch.entry_arm in lm.arms:   # arm_only without prediction (sur) → fallback
                exit_arm = turns.get((ch.entry_arm, "through"))  # default route: does NOT arrive at the stop line
        exit_edge = lm.arms[exit_arm].exit_edge if exit_arm in lm.arms else None
        # OBSERVED EXIT LANE (exit bands, same method as entry): for the DATA-DEPENDENT egress
        # in build_path (chooses the real net connection whose egress = this lane,
        # instead of the FIXED egress of junction_internal). '' / None if the exit approach is not resolvable.
        exit_band = lane_bands.get("exit", {}).get(exit_arm)
        exit_carril = None
        if exit_band and exit_arm in stop_lines:
            chdf = pd.DataFrame({"gx": ch.gx, "gy": ch.gy})
            s_exit = dte.lane_cross_coord(chdf, stop_lines[exit_arm], center, kind="exit")
            ex = dte.classify_lane(s_exit, exit_band)
            exit_carril = int(ex) if ex.isdigit() else None
        # CLASSIFIED entry lane (the lateral decomposition). Group "2-3" (oeste): the camera does not
        # separate 2 from 3 (sensor limit).
        from digital_shadow.sumo.position import resolve_lane
        ocommit = next((c for c in ch.route.commits() if c.field == "origin"), None)
        lane_label = str(ocommit.lane_label) if ocommit else ""
        group_carriles = [int(p) for p in lane_label.split("-")] if all(p.isdigit() for p in lane_label.split("-") if p) and lane_label else [1]
        # EGRESS-BASED DISAMBIGUATION (GENERAL geometric mechanism, not a west table): in COMPLETE, the
        # observed TURN constrains the grouped lane — left → LEFTMOST lane
        # (maximum, because lane 1=right), right → RIGHTMOST (minimum). STRAIGHT does not
        # disambiguate → it stays at the MEAN of the group (declared sensor limit). It is OBSERVED info
        # (high delay); in the drawing it does not matter, in the causal measurement (step 3) it will be an observed commit.
        if complete and len(group_carriles) > 1:
            turn = next((t for (ea, t), x in turns.items() if ea == ch.entry_arm and x == exit_arm), None)
            if turn == "left":
                group_carriles = [max(group_carriles)]      # leftmost end of the group
            elif turn == "right":
                group_carriles = [min(group_carriles)]      # rightmost end of the group
            # through → the whole group stays (mean); the sensor does not distinguish it
        entry_lane_id, _mode = resolve_lane(lm, ch.entry_arm, lane_label) if ch.entry_arm in lm.arms else (None, None)
        dist_center = np.hypot(np.asarray(ch.gx) - center[0], np.asarray(ch.gy) - center[1])
        # lag = INSTANTIATION delay Section 7.2 (positive: SUMO AFTER the physical event). It is small
        # (~0.4 s): the shadow's position is low delay (faithful). The gap time is NOT added
        # (that is CONTINUITY, not appearance delay; the orchestrator instantiates at the crossing regardless).
        veh[ch.vid] = dict(t=np.array(ch.ts), xy=world, entry_arm=ch.entry_arm, exit_arm=exit_arm,
                           cls=COCO_NAME.get(ch.running_class(), "car"),
                           entry_edge=lm.arms[ch.entry_arm].entry_edge if ch.entry_arm in lm.arms else None,
                           exit_edge=exit_edge, entry_lane_id=entry_lane_id, cross_t=ch.cross_t,
                           approach_axis=lm.approach_axis.get(ch.entry_arm),
                           lane_label=lane_label, group_carriles=group_carriles, dist_center=dist_center,
                           exit_carril=exit_carril, delay=inst_delay.get(ch.root, 0.37))
    return veh, H


def render(veh, t0, t1, out):
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    os.environ.setdefault("DISPLAY", ":1")
    import traci
    from digital_shadow.sumo.stage import SumoStage
    SHOTS.mkdir(parents=True, exist_ok=True)
    for f in SHOTS.glob("*.png"):
        f.unlink()
    cap = cv2.VideoCapture(str(ROOT / "data/sources/rusia/rusia_tramo_000-180.mp4"))
    stage = SumoStage(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml",
                      gui=True, step_length=0.001).start()
    tr = stage._traci
    define_vtypes(tr)
    tr.gui.setSchema("View #0", "real world")
    tr.gui.setZoom("View #0", 700)
    tr.gui.setOffset("View #0", 57.0, 74.4)
    # STEP 1: reconstruction by LANE GEOMETRY (3 segments). Per vehicle, the trajectory over the
    # net geometry (approach=classified lane, junction=internal curve of the OBSERVED turn with projected
    # advance that respects stops, egress=exit lane). Uniform moveToXY(recon, keepRoute=2) → no
    # ~1-lane drift, no transition jump, and the turn is reconstructed.
    lm_r = load_lane_map("rusia")
    H_pres = load_homography("rusia")          # only for the PRESENTATION dead-band (image px)
    deadband_px, median_window_s = load_presentation("rusia")
    for v in veh.values():
        v["depart_lane"] = 0                                   # default if the connection is not resolvable
        if v["entry_arm"] in lm_r.arms and v["exit_arm"] in lm_r.arms:
            try:
                # ONE unified connection: recon AND routing (departLane) share it → same entry+internal+
                # egress, recon-vs-SIM mismatch ~0. departLane = from_idx of THAT connection (observed lane
                # in LEGAL/ROUTABLE; canonical in ORPHAN → never jump #7).
                conn = choose_connection(tr, lm_r, v["entry_arm"], v["exit_arm"], v["group_carriles"],
                                         exit_carril=v.get("exit_carril"))
                if conn is not None:
                    v["depart_lane"] = int(conn["from_idx"])
                path = build_path(tr, lm_r, v["entry_arm"], v["exit_arm"], v["group_carriles"],
                                  exit_carril=v.get("exit_carril"), conn=conn)
                v["recon_t"], v["recon_xy"] = reconstruct_world(v["xy"], v["t"], path)
            except Exception:
                v["recon_t"], v["recon_xy"] = v["t"], v["xy"]   # incomplete geometry → raw trail
        else:
            v["recon_t"], v["recon_xy"] = v["t"], v["xy"]
        # PRESENTATION: SMOOTHED position (dead-band) ONLY for the demo drawing. The fidelity
        # metrics (Section 7) are measured in SEPARATE loops that use RAW recon_xy; render() emits no metrics.
        v["draw_xy"] = presentation_smooth(np.asarray(v["recon_xy"], float), np.asarray(v["recon_t"], float),
                                           H_pres, deadband_px, median_window_s)
    live = {}                              # vid -> True if it is in SUMO
    STEP = 3                               # samples every 3 video frames (~10 fps screencast)
    nshot = 0
    f0, f1 = int(t0 / DT), int(t1 / DT)
    for fi in range(f0, f1, STEP):
        t = fi * DT                        # VIDEO time (master clock)
        active_delays = []
        for vid, v in veh.items():
            # CORRECT lag: at video time t, SUMO shows the position from `delay` ago (the
            # PAST, t−delay). The vehicle appears in SUMO at entry+delay (AFTER the video) →
            # video AHEAD, consistent with the positive delays of Section 7.2.
            tshift = t - v["delay"]
            if v["t"][0] <= tshift <= v["t"][-1]:
                x = float(np.interp(tshift, v["t"], v["xy"][:, 0]))
                y = float(np.interp(tshift, v["t"], v["xy"][:, 1]))
                active_delays.append(v["delay"])
                if vid not in live:
                    if v["entry_edge"] is None:
                        continue
                    stage.add(vid, v["entry_edge"], v.get("depart_lane", 0), 5.0,
                              vtype=VTYPE.get(v["cls"], "DEFAULT_VEHTYPE"))
                    if v["exit_edge"]:
                        # route BEYOND the junction (issue 6140): if the egress has a successor, aim
                        # there → the vehicle does not 'arrive' at the end of the egress.
                        suc = next((l[0].rsplit("_", 1)[0] for l in tr.lane.getLinks(v["exit_edge"] + "_0")), None)
                        v["_tgt"] = suc or v["exit_edge"]
                        try: stage.set_target(vid, v["_tgt"])
                        except Exception:
                            v["_tgt"] = v["exit_edge"]
                            try: stage.set_target(vid, v["exit_edge"])
                            except Exception: pass
                    live[vid] = True
                if vid in live:
                    # FRONT 1: the position is dictated by moveToXY. keepRoute=1 (maps to the ROUTE, does not reset
                    # the route to single-edge like keepRoute=2 when entering the internal lane → disappearance).
                    # If the route still collapses, it is RE-SET (official workaround: keep route forward).
                    # PRESENTATION FRONT: the SMOOTHED trail (draw_xy) is drawn, not the raw recon. The
                    # MEASURED shadow (Section 7, separate loops) still reads raw recon_xy → fidelity intact.
                    rx = float(np.interp(tshift, v["recon_t"], v["draw_xy"][:, 0]))
                    ry = float(np.interp(tshift, v["recon_t"], v["draw_xy"][:, 1]))
                    try:
                        tr.vehicle.moveToXY(vid, "", -1, rx, ry, keepRoute=1)
                        if v.get("_tgt") and len(tr.vehicle.getRoute(vid)) < 2:
                            try: tr.vehicle.changeTarget(vid, v["_tgt"])
                            except Exception: pass
                        # POSITION-DRIVEN: the implicit velocity of moveToXY (large if the vehicle
                        # APPEARS far from departPos → jump) would throw the vehicle out of the network in the
                        # step. setSpeed(0): WE dictate the position, it does not advance on its own.
                        tr.vehicle.setSpeed(vid, 0.0)
                    except Exception:
                        live.pop(vid, None)
            elif vid in live and tshift > v["t"][-1]:
                try: stage.remove(vid)      # CLEAN retirement at the end of its trajectory (it left)
                except Exception: pass
                live.pop(vid, None)
        tr.simulationStep(t)                # SUMO clock = t_video (lag in SPACE, not in the clock)
        shot = SHOTS / f"sumo_{nshot:04d}.png"
        tr.gui.screenshot("View #0", str(shot))
        tr.simulationStep(t + 0.001)
        compose(cap, fi, shot, t, active_delays, SHOTS / f"frame_{nshot:04d}.png")
        nshot += 1
    stage.close(); cap.release()
    # screencast at ~real speed (STEP video frames per shot → fps = 29.583/STEP)
    fps = 29.583 / STEP
    subprocess.run(["ffmpeg", "-y", "-framerate", f"{fps:.2f}", "-i", str(SHOTS / "frame_%04d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out)],
                   check=False, capture_output=True)
    return nshot


def compose(cap, fi, sumo_png, t, delays, out):
    cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
    ok, vid_frame = cap.read()
    sumo = cv2.imread(str(sumo_png))
    if not ok or sumo is None:
        return
    h = 480
    vid_frame = cv2.resize(vid_frame, (int(1200 * h / 646), h))
    sumo = cv2.resize(sumo, (int(sumo.shape[1] * h / sumo.shape[0]), h))
    canvas = cv2.hconcat([vid_frame, sumo])
    W = canvas.shape[1]
    d = np.array(delays) if delays else np.array([0.0])
    # panel labels
    cv2.rectangle(canvas, (0, 0), (W, 58), (0, 0, 0), -1)
    cv2.putText(canvas, "VIDEO (realidad)", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(canvas, "SUMO (sombra; rezago espacial = delay (7.2))", (vid_frame.shape[1] + 10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(canvas,
                f"t_video={t:5.1f}s   activos={len(delays)}   delay: mediana {np.median(d):.2f}s "
                f"[{np.min(d):.2f}-{np.max(d):.2f}s]  (predicho sigue de cerca; sur/fragmentado mas rezagado)",
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imwrite(str(out), canvas)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--t0", type=float, default=40.0)
    ap.add_argument("--t1", type=float, default=80.0)
    ap.add_argument("--out", default=str(ROOT / "verify_out/harness_viz_screencast.mp4"))
    args = ap.parse_args()
    print("Corriendo el núcleo causal (medición, RecordingSink) para obtener stream + delays...")
    veh, H = run_core()
    print(f"  {len(veh)} vehículos instanciados; render del tramo [{args.t0}, {args.t1}] s a sumo-gui...")
    n = render(veh, args.t0, args.t1, args.out)
    print(f"  {n} frames compuestos → screencast: {args.out}")
    print(f"  (ver el screencast; vehículos en SUMO siguiendo a los del video con su rezago,")
    print(f"   heterogéneo; el sur más rezagado/a nivel brazo; sin teletransporte donde el relleno actúa.)")


if __name__ == "__main__":
    main()

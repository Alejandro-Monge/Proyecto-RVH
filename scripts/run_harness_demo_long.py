"""GENERALIZATION / CAUSALITY TEST — the same causal pipeline with the SOURCE swapped and NOTHING ELSE.

Video: data/sources/rusia_long/rusia_long.mp4 = the complete ORIGINAL RAW (byte-identical to
interseccion_rusia2.mp4, md5 c95e0d58; VFR, 282.727 s). Its first 180 s ARE the segment on
which everything was built; the genuinely UNSEEN part is the final ~103 s. It is declared: the
test validates (a) the end-to-end causal pipeline from raw pixels over a longer
horizon, with ~103 s never seen, and (b) that NOTHING is re-tuned.

CENTRAL RULE (what this test validates): the pipeline, the calibration and ALL the constants
are INHERITED intact — homography, zones, bands, detector/tracker/reconnection/thresholds,
reconstruct.py, net. The provenance guard verifies the md5s BEFORE running and this script
does NOT accept any tuning parameter. Any new case the long video brings is let run as-is:
a failure here is a RESULT, not something to fix.

ONLY PREPROCESSING (conditioning of the input signal, source interface): force CFR,
with the SAME ffmpeg recipe as the segment cut but without the -t limit: libx264 crf 18
re-encoding that emits the CFR 355/12 grid
(the raw is VFR avg 29.5656). It is what was done to the 180 s one; not one filter more.

STAGES (each a faithful replica of the canonical script, with ONLY the source/output path
substituted — the project's source abstraction exists exactly for this):
  1. condition    → data/sources/rusia_long/rusia_long_cfr355-12.mp4
  2. canonical real-time tracking (replica of run_tracker.py, default mode)
                   → verify_out/generalizacion_long/tracks_long.csv
  3. events       → scripts/derive_track_events.py (production CLI, intact)
                   → verify_out/generalizacion_long/track_events_long.csv
  4. causal core (replica of run_core in run_harness_viz.py with substituted paths and NO GT:
     the long video has no ground truth → instantiation delay against its own crossing,
     the same fallback run_core already uses for the unmatched)
  5. output: split VIDEO|SUMO mp4 (format of the annex demo) → verify_out/generalizacion_long/
     demo_split_long.mp4;  or  --live (live demo: ffplay+sumo-gui paced to wall-clock).

Usage: python scripts/run_harness_demo_long.py            # stages 1-5, split mp4
       python scripts/run_harness_demo_long.py --live     # stages 1-4 + live demo
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common  # noqa: E402  (provenance guard)

RAW = ROOT / "data/sources/rusia_long/rusia_long.mp4"
CFR = ROOT / "data/sources/rusia_long/rusia_long_cfr355-12.mp4"
OUT_DIR = ROOT / "verify_out/generalizacion_long"
TRACKS = OUT_DIR / "tracks_long.csv"
EVENTS = OUT_DIR / "track_events_long.csv"
OUT_MP4 = OUT_DIR / "demo_split_long.mp4"

# ALL the inherited items the guard verifies BEFORE running (code + calibration + configs).
INPUTS = [
    "digital_shadow/sumo/reconstruct.py",
    "digital_shadow/perception/detector.py",
    "config/sites/rusia/sumo/intersection_rusia.net.xml",
    "config/detector.yaml", "config/tracker.yaml", "config/positioning.yaml",
    "config/sites/rusia/site.yaml", "config/sites/rusia/zones.yaml",
    "config/sites/rusia/sumo/lane_map.yaml", "config/sites/rusia/lane_turn_rule.yaml",
    "config/sites/rusia/turns.csv",
    "verify_out/homography_gcp/homography_img2ground.npy",
    "scripts/run_harness_viz.py", "scripts/derive_track_events.py",
]

VID_W, VID_H, SUMO_W, HUD_H, FPS = 1200, 646, 1148, 64, 29.583
SHOTS = Path(os.environ.get("SHOTS_DIR", "/tmp/gemelo_rvh_scratch")) / "long_shots"


# --------------------------------------------------------------------------- #
# 1) Source conditioning: SAME recipe as the segment cut, without -t
# --------------------------------------------------------------------------- #
def condition_source() -> None:
    if CFR.exists():
        print(f"[1] fuente CFR ya existe: {CFR.name}")
        return
    print("[1] acondicionando fuente (receta del tramo, sin límite de tiempo)…")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "warning", "-stats", "-i", str(RAW),
                    "-map", "0:v:0", "-c:v", "libx264", "-crf", "18", "-preset", "medium",
                    "-an", str(CFR)], check=True)
    probe = subprocess.run(["ffprobe", "-v", "error", "-show_entries",
                            "stream=r_frame_rate,avg_frame_rate,nb_frames",
                            "-of", "default=noprint_wrappers=1", str(CFR)],
                           capture_output=True, text=True).stdout
    print(f"    verificación CFR:\n{probe}")
    if "r_frame_rate=355/12" not in probe or "avg_frame_rate=355/12" not in probe:
        raise RuntimeError("la salida NO quedó CFR 355/12 — PARAR y reportar (no ajustar solo)")


# --------------------------------------------------------------------------- #
# 2) Canonical real-time tracking — replica of run_tracker.py with the source substituted
# --------------------------------------------------------------------------- #
def run_tracking() -> None:
    if TRACKS.exists():
        print(f"[2] tracks ya existen: {TRACKS.name}")
        return
    print("[2] tracking canónico real-time (detector→ByteTrack, fuente sustituida)…")
    from digital_shadow.config import (load_detector_config, load_site_config,
                                       load_tracker_config)
    from digital_shadow.perception import YoloDetector
    from digital_shadow.sources import FileVideoSource
    from digital_shadow.tracking import reconnector_from_config, tracker_from_config
    from run_tracker import CSV_HEADER

    site = load_site_config("rusia")
    det_cfg = load_detector_config()
    trk_cfg = load_tracker_config()
    fps = float(site.fps_ref)
    detector = YoloDetector(det_cfg, list(site.class_set)).load()
    detector.warmup(site.frame_height, site.frame_width)
    tracker = tracker_from_config(trk_cfg, site)
    reconnector = reconnector_from_config(trk_cfg, site)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    n_rows, counts = 0, Counter()
    with open(TRACKS, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(CSV_HEADER)
        # ONLY change vs run_tracker: the video PATH (same class, same fps_ref and
        # expected site resolution — the project's source abstraction).
        source = FileVideoSource(video_path=CFR, fps_ref=site.fps_ref,
                                 expected_resolution=site.resolution)
        with source:
            while True:
                frame = source.read()
                if frame.end_of_source:
                    break
                ts = frame.timestamp_s
                fidx = round(ts * fps)
                tracked = tracker.update(detector.detect(frame.image))
                tracked = reconnector.apply(tracked, frame.image, fidx)
                for t in tracked:
                    writer.writerow([f"{ts:.6f}", fidx, t.track_id,
                                     f"{t.x1:.2f}", f"{t.y1:.2f}", f"{t.x2:.2f}", f"{t.y2:.2f}",
                                     f"{t.cx:.2f}", f"{t.cy:.2f}", t.class_id,
                                     f"{t.confidence:.4f}"])
                    counts[t.track_id] += 1
                    n_rows += 1
    print(f"    {n_rows} filas, {len(counts)} tracks únicos → {TRACKS.name}")


# --------------------------------------------------------------------------- #
# 3) Per-track events — production CLI intact (paths only)
# --------------------------------------------------------------------------- #
def run_events() -> None:
    if EVENTS.exists():
        print(f"[3] eventos ya existen: {EVENTS.name}")
        return
    print("[3] derivando eventos (derive_track_events.py de producción, zonas/bandas heredadas)…")
    subprocess.run([sys.executable, str(ROOT / "scripts/derive_track_events.py"),
                    "--site", "rusia", "--tracks", str(TRACKS), "--out", str(EVENTS)],
                   check=True, cwd=ROOT)


# --------------------------------------------------------------------------- #
# 4) Causal core — replica of run_core (run_harness_viz.py) with substituted paths, NO GT
# --------------------------------------------------------------------------- #
def run_core_long():
    print("[4] núcleo causal (réplica de run_core, fuente larga, sin GT)…")
    import derive_track_events as dte
    import reconnect_fragments as rf
    from digital_shadow.config.site import load_site_config
    from digital_shadow.geometry.ground_contact import add_ground_contact
    from digital_shadow.harness.orchestrator import CausalOrchestrator
    from digital_shadow.harness.sink import RecordingSink
    from digital_shadow.sumo.lane_map import load_lane_map
    from digital_shadow.sumo.position import load_alpha, load_homography, project_to_world, resolve_lane
    from digital_shadow.sumo.route import load_lane_turn_rule, load_turns, predict_turn
    from digital_shadow.tracking.causal_reconnect import CausalReconnector
    from audit_harness_part3 import internal_shapes_from_net
    from run_harness_viz import COCO_NAME

    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    tracks = pd.read_csv(TRACKS); add_ground_contact(tracks)
    base = pd.read_csv(EVENTS)
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
    inst_t = {c.chain: c.t_commit for c in commits if c.kind == "instanciacion"}
    cross_t = {c.chain: (c.payload.get("cross_t") or c.t_commit) for c in commits
               if c.kind == "instanciacion"}
    # NO GT (the long video has none): delay against its OWN crossing — the same fallback that
    # run_core applies to the unmatched. Nothing else changes.
    inst_delay = {root: max(0.1, tc - cross_t[root]) for root, tc in inst_t.items()}

    veh = {}
    for ch in chains.values():
        if not ch.instantiated:
            continue
        world = np.array([project_to_world(H, (gx, gy)) for gx, gy in zip(ch.gx, ch.gy)])
        tail = np.array([np.median(ch.gx[-dte.WINDOW_FRAMES:]), np.median(ch.gy[-dte.WINDOW_FRAMES:])])
        xa = dte.arm_of(tail, center, dte.CORE_RADIUS_PX)
        complete = xa in lm.arms and xa != ch.entry_arm
        if complete:
            exit_arm = xa
        else:
            ocom = next((c for c in ch.route.commits() if c.field == "origin"), None)
            pt = predict_turn(rule, ch.entry_arm, ocom.lane_label if ocom else "") if ch.entry_arm in lm.arms else None
            exit_arm = turns.get((ch.entry_arm, pt)) if pt else None
            if exit_arm is None and ch.entry_arm in lm.arms:
                exit_arm = turns.get((ch.entry_arm, "through"))
        exit_edge = lm.arms[exit_arm].exit_edge if exit_arm in lm.arms else None
        exit_band = lane_bands.get("exit", {}).get(exit_arm)
        exit_carril = None
        if exit_band and exit_arm in stop_lines:
            chdf = pd.DataFrame({"gx": ch.gx, "gy": ch.gy})
            s_exit = dte.lane_cross_coord(chdf, stop_lines[exit_arm], center, kind="exit")
            ex = dte.classify_lane(s_exit, exit_band)
            exit_carril = int(ex) if ex.isdigit() else None
        ocommit = next((c for c in ch.route.commits() if c.field == "origin"), None)
        lane_label = str(ocommit.lane_label) if ocommit else ""
        group_carriles = [int(p) for p in lane_label.split("-")] \
            if all(p.isdigit() for p in lane_label.split("-") if p) and lane_label else [1]
        if complete and len(group_carriles) > 1:
            turn = next((t for (ea, t), x in turns.items() if ea == ch.entry_arm and x == exit_arm), None)
            if turn == "left":
                group_carriles = [max(group_carriles)]
            elif turn == "right":
                group_carriles = [min(group_carriles)]
        entry_lane_id, _mode = resolve_lane(lm, ch.entry_arm, lane_label) if ch.entry_arm in lm.arms else (None, None)
        veh[ch.vid] = dict(t=np.array(ch.ts), xy=world, entry_arm=ch.entry_arm, exit_arm=exit_arm,
                           cls=COCO_NAME.get(ch.running_class(), "car"),
                           entry_edge=lm.arms[ch.entry_arm].entry_edge if ch.entry_arm in lm.arms else None,
                           exit_edge=exit_edge, entry_lane_id=entry_lane_id, cross_t=ch.cross_t,
                           lane_label=lane_label, group_carriles=group_carriles,
                           exit_carril=exit_carril, delay=inst_delay.get(ch.root, 0.37))
    print(f"    {len(veh)} vehículos instanciados")
    return veh


# --------------------------------------------------------------------------- #
# 5) Outputs: split mp4 (format of the annex demo) or live demo (mirror of run_harness_demo)
# --------------------------------------------------------------------------- #
def prepare_recon(veh, tr):
    """Reconstruction block identical to render()/demo (production)."""
    from digital_shadow.sumo.lane_map import load_lane_map
    from digital_shadow.sumo.position import load_homography
    from digital_shadow.sumo.presentation import load_presentation, presentation_smooth
    from digital_shadow.sumo.reconstruct import build_path, choose_connection, reconstruct_world
    lm_r = load_lane_map("rusia")
    H_pres = load_homography("rusia")
    deadband_px, median_window_s = load_presentation("rusia")
    for v in veh.values():
        v["depart_lane"] = 0
        if v["entry_arm"] in lm_r.arms and v["exit_arm"] in lm_r.arms:
            try:
                conn = choose_connection(tr, lm_r, v["entry_arm"], v["exit_arm"],
                                         v["group_carriles"], exit_carril=v.get("exit_carril"))
                if conn is not None:
                    v["depart_lane"] = int(conn["from_idx"])
                path = build_path(tr, lm_r, v["entry_arm"], v["exit_arm"], v["group_carriles"],
                                  exit_carril=v.get("exit_carril"), conn=conn)
                v["recon_t"], v["recon_xy"] = reconstruct_world(v["xy"], v["t"], path)
            except Exception:
                v["recon_t"], v["recon_xy"] = v["t"], v["xy"]
        else:
            v["recon_t"], v["recon_xy"] = v["t"], v["xy"]
        v["draw_xy"] = presentation_smooth(np.asarray(v["recon_xy"], float),
                                           np.asarray(v["recon_t"], float),
                                           H_pres, deadband_px, median_window_s)


def place_frame(stage, tr, veh, live, t, VTYPE):
    """Per-frame placement — identical to production render()/demo (kr1, set_target, speed 0)."""
    active = []
    for vid, v in veh.items():
        tshift = t - v["delay"]
        if v["t"][0] <= tshift <= v["t"][-1]:
            active.append(v["delay"])
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
                rx = float(np.interp(tshift, v["recon_t"], v["draw_xy"][:, 0]))
                ry = float(np.interp(tshift, v["recon_t"], v["draw_xy"][:, 1]))
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
    return active


def hud(canvas, t, delays):
    W = canvas.shape[1]
    cv2.rectangle(canvas, (0, 0), (W, HUD_H), (12, 12, 12), -1)
    cv2.putText(canvas, "VIDEO LARGO (realidad; 0-180s = tramo visto, >180s = NO visto)",
                (14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "SOMBRA EN SUMO (pipeline causal HEREDADO, cero ajustes)",
                (VID_W + 14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (90, 220, 255), 2, cv2.LINE_AA)
    d = np.array(delays) if delays else np.array([0.0])
    marca = "TRAMO VISTO (calibracion)" if t < 180.0 else ">>> SEGMENTO NO VISTO <<<"
    cv2.putText(canvas,
                f"t_video = {t:6.2f} s   [{marca}]   activos: {len(delays):2d}   "
                f"rezago causal: mediana {np.median(d):.2f} s [{d.min():.2f} - {d.max():.2f} s]",
                (14, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (170, 255, 170), 1, cv2.LINE_AA)
    cv2.line(canvas, (VID_W, 0), (VID_W, canvas.shape[0]), (12, 12, 12), 3)


def render_split(veh) -> int:
    from digital_shadow.sumo.stage import SumoStage
    from run_harness_viz import DT, VTYPE, define_vtypes
    SHOTS.mkdir(parents=True, exist_ok=True)
    for f in SHOTS.glob("*.png"):
        f.unlink()
    cap = cv2.VideoCapture(str(CFR))
    stage = SumoStage(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml",
                      gui=True, step_length=0.001).start()
    tr = stage._traci
    define_vtypes(tr)
    tr.gui.setSchema("View #0", "real world")
    tr.gui.setZoom("View #0", 700)
    tr.gui.setOffset("View #0", 57.0, 74.4)
    prepare_recon(veh, tr)

    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cw, chh = VID_W + SUMO_W, VID_H + HUD_H
    ff = subprocess.Popen(["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                           "-s", f"{cw}x{chh}", "-r", f"{FPS:.3f}", "-i", "-",
                           "-c:v", "libx264", "-crf", "18", "-preset", "medium",
                           "-pix_fmt", "yuv420p", str(OUT_MP4)], stdin=subprocess.PIPE,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    live, written = {}, 0
    for fi in range(n_frames):
        ok, vid_frame = cap.read()
        if not ok:
            break
        t = fi * DT
        active = place_frame(stage, tr, veh, live, t, VTYPE)
        tr.simulationStep(t)
        shot = SHOTS / f"s_{fi % 8:d}.png"
        tr.gui.screenshot("View #0", str(shot), width=SUMO_W, height=VID_H)
        tr.simulationStep(t + 0.001)
        sumo_img = cv2.imread(str(shot))
        if sumo_img is None:
            continue
        if sumo_img.shape[:2] != (VID_H, SUMO_W):
            sumo_img = cv2.resize(sumo_img, (SUMO_W, VID_H))
        canvas = np.zeros((chh, cw, 3), np.uint8)
        canvas[HUD_H:, :VID_W] = vid_frame
        canvas[HUD_H:, VID_W:] = sumo_img
        hud(canvas, t, active)
        ff.stdin.write(canvas.tobytes())
        written += 1
        if written % 1000 == 0:
            print(f"    {written} frames ({t:.1f}s)…", flush=True)
    ff.stdin.close(); ff.wait(); stage.close(); cap.release()
    return written


def run_live(veh) -> None:
    """Live loop (ffplay + pacing to wall-clock)."""
    from digital_shadow.sumo.stage import SumoStage
    from run_harness_viz import DT, VTYPE, define_vtypes
    stage = SumoStage(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml",
                      gui=True, step_length=0.001).start()
    tr = stage._traci
    define_vtypes(tr)
    tr.gui.setSchema("View #0", "real world")
    tr.gui.setZoom("View #0", 700)
    tr.gui.setOffset("View #0", 57.0, 74.4)
    prepare_recon(veh, tr)
    proc = subprocess.Popen(["ffplay", "-an", "-autoexit", "-loglevel", "quiet",
                             "-window_title", "VIDEO LARGO (realidad)", "-x", "900", str(CFR)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.6)
    live = {}
    T0 = time.perf_counter()
    n = int(282.7 / DT)
    for fi in range(n):
        t_v = fi * DT
        place_frame(stage, tr, veh, live, t_v, VTYPE)
        tr.simulationStep(t_v)
        target = T0 + t_v
        now = time.perf_counter()
        if now < target:
            time.sleep(target - now)
    stage.close()
    proc.terminate()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--live", action="store_true", help="demo en vivo (ffplay+sumo-gui) en vez de mp4")
    args = ap.parse_args()
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    os.environ.setdefault("DISPLAY", ":1")

    stamp = common.check_provenance(INPUTS)          # INHERITED intact or it does not run
    print(f"[0] guardia de procedencia OK ({len(stamp)} insumos verificados)")
    condition_source()
    run_tracking()
    run_events()
    veh = run_core_long()

    if args.live:
        run_live(veh)
        return
    written = render_split(veh)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "provenance_generalizacion.txt").write_text("\n".join([
        "prueba de GENERALIZACIÓN/CAUSALIDAD — fuente sustituida, pipeline HEREDADO intacto",
        f"fuente cruda: {RAW.name} md5 c95e0d584455bd0198b6eb654e128fd2 (= crudo original completo; "
        "0-180s = tramo visto, ~180-282.7s = NO visto)",
        "preproceso: SOLO re-encode CFR 355/12 (receta del recorte del tramo, sin -t)",
        "sin GT: delay de instanciación contra cruce propio (fallback estándar de run_core)",
        f"frames escritos: {written}",
        *(f"  {p} = {m}" for p, m in sorted(stamp.items())),
    ]) + "\n")
    print(f"[ok] {OUT_MP4.relative_to(ROOT)} ({written} frames)")


if __name__ == "__main__":
    main()

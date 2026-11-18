"""IMPLEMENTACIÓN Multi-Cámara (Re-ID cinemático + Car-Following en punto ciego) para `scripts/run_multicam_harness.py`.

Lee exclusivamente `verify_out/events/merged/track_events_merged.csv` (y las trayectorias crudas
de ambas cámaras únicamente para proyectar la posición cuadro a cuadro cuando el vehículo está
en campo de visión):
  - `global_*` (fusionados): inyección en Cámara 1 -> control perceptual (`moveToXY`) en Cam 1 ->
    tramo ciego (`334948975#2`, `334948975#3`) conducido por el modelo Car-Following de SUMO
    (`setSpeedMode(31)`, `setSpeed(-1)`) sin remover el vehículo -> resincronización en Cámara 2
    (`moveToXY`) -> retiro al salir de Cam 2.
  - `c1_*` / `c2_*` (huérfanos de una sola cámara): inyección, movimiento local y retiro en su
    propia intersección.

Vista de `sumo-gui`: centrada en `(258.64, 91.22)` con `zoom=300` para abarcar ambas intersecciones
(`453731125` y `577206092`) y el corredor conector simultáneamente.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from digital_shadow.config.site import load_site_config  # noqa: E402
from digital_shadow.geometry.ground_contact import add_ground_contact  # noqa: E402
from digital_shadow.sumo.lane_map import load_lane_map  # noqa: E402
from digital_shadow.sumo.position import load_homography, project_to_world  # noqa: E402
from digital_shadow.sumo.presentation import load_presentation, presentation_smooth  # noqa: E402
from digital_shadow.sumo.reconstruct import _densify, _nearest_arclen  # noqa: E402
from digital_shadow.sumo.stage import SumoStage, point_at_arclen, polyline_arclen  # noqa: E402

DEFAULT_MERGED_CSV = ROOT / "verify_out/events/merged/track_events_merged.csv"
NET_PATH = ROOT / "config/sites/cuenca/sumo/zona_final.net.xml"
CAM1_SITE = "cuenca_oeste"
CAM2_SITE = "cuenca"
CAM1_TRACKS_CSV = ROOT / "verify_out/tracks_cuenca_oeste.csv"
CAM2_TRACKS_CSV = ROOT / "verify_out/tracks_cuenca.csv"
OUT_DIR = ROOT / "verify_out/demo_causal"
SHOTS = Path(os.environ.get("SHOTS_DIR", "/tmp/gemelo_rvh_scratch")) / "multicam_shots"

# Centro geométrico entre el nodo 453731125 (174.83, 100.82) y el nodo 577206092 (342.45, 81.61)
GUI_OFFSET_X = 258.64
GUI_OFFSET_Y = 91.22
GUI_ZOOM = 300

VTYPE = {"car": "viz_car", "bus": "viz_bus", "truck": "viz_truck"}
_VTYPE_VCLASS = {"viz_car": "car", "viz_bus": "bus", "viz_truck": "truck"}
VAR_POSITION, VAR_ANGLE, VAR_TYPE = 0x42, 0x43, 0x4f
_SUB_VARS = [VAR_POSITION, VAR_ANGLE, VAR_TYPE]
SUMO_W, HUD_H = 1280, 64
TIMEOUT_S = 0.5


def _load_tracks_by_id(csv_path: Path, H: np.ndarray) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    df = pd.read_csv(csv_path)
    add_ground_contact(df)
    out: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for tid, g in df.groupby("track_id"):
        g = g.sort_values("frame_index")
        ts = g["timestamp_s"].to_numpy(float)
        xy = np.array([project_to_world(H, (r.gx, r.gy)) for r in g.itertuples()], dtype=float)
        out[int(tid)] = (ts, xy)
    return out


def _build_edge_path(tr, route_edges: list[str]) -> np.ndarray | None:
    """Construye una polilínea continua en el mundo siguiendo las lanes de `route_edges`."""
    if not route_edges:
        return None
    parts: list[np.ndarray] = []
    for idx, edge in enumerate(route_edges):
        try:
            lane0 = np.asarray(tr.lane.getShape(f"{edge}_0"), float)
        except Exception:
            continue
        parts.append(lane0)
        if idx + 1 < len(route_edges):
            nxt = route_edges[idx + 1]
            # Buscar la lane interna que conecta `edge` -> `nxt`
            via_id = None
            n_lanes = tr.edge.getLaneNumber(edge)
            for li in range(n_lanes):
                for lk in tr.lane.getLinks(f"{edge}_{li}"):
                    if lk[0].rsplit("_", 1)[0] == nxt and lk[4]:
                        via_id = lk[4]
                        break
                if via_id:
                    break
            if via_id:
                cur = via_id
                seen = set()
                while cur and cur.startswith(":") and cur not in seen:
                    seen.add(cur)
                    parts.append(np.asarray(tr.lane.getShape(cur), float))
                    links = tr.lane.getLinks(cur)
                    cur = links[0][0] if links else None
    if not parts:
        return None
    merged = [parts[0]]
    for p in parts[1:]:
        if np.hypot(*(p[0] - merged[-1][-1])) < 0.05:
            p = p[1:]
        if len(p):
            merged.append(p)
    return _densify(np.vstack(merged), step=1.0)


def _snap_to_path(raw_xy: np.ndarray, ts: np.ndarray, path: np.ndarray | None,
                  H: np.ndarray, deadband_px: float, median_window_s: float) -> np.ndarray:
    """Proyecta monótonamente `raw_xy` sobre la polilínea de la ruta y suaviza el jitter."""
    if path is None or len(path) < 2 or len(raw_xy) == 0:
        return raw_xy
    arc = polyline_arclen(path)
    L = float(arc[-1])
    s = np.zeros(len(raw_xy))
    s[0] = _nearest_arclen(path, arc, raw_xy[0], 0.0, L)
    for i in range(1, len(raw_xy)):
        s[i] = _nearest_arclen(path, arc, raw_xy[i], s[i - 1], L)
    snapped = np.array([point_at_arclen(path, float(si)) for si in s])
    return presentation_smooth(snapped, ts, H, deadband_px, median_window_s)


def build_multicam_vehicles(merged_csv: Path = DEFAULT_MERGED_CSV) -> dict[str, dict]:
    """Carga los vehículos desde `track_events_merged.csv` y los empareja con sus trayectorias."""
    df = pd.read_csv(merged_csv)
    H1 = load_homography(CAM1_SITE)
    H2 = load_homography(CAM2_SITE)
    t1_map = _load_tracks_by_id(CAM1_TRACKS_CSV, H1)
    t2_map = _load_tracks_by_id(CAM2_TRACKS_CSV, H2)

    veh: dict[str, dict] = {}
    for r in df.itertuples():
        route_edges = json.loads(r.route_edges) if isinstance(r.route_edges, str) else []
        if not route_edges:
            continue  # p.ej. c1_29 / c1_38 sin entry_edge válido en mapa unidireccional

        gid = str(r.global_id)
        status = str(r.status)
        vcls = str(r.vehicle_class) if pd.notna(r.vehicle_class) and r.vehicle_class else "car"

        seg1 = t1_map.get(int(r.cam1_track_id)) if pd.notna(r.cam1_track_id) else None
        seg2 = t2_map.get(int(r.cam2_track_id)) if pd.notna(r.cam2_track_id) else None

        veh[gid] = {
            "id": gid,
            "status": status,
            "cls": vcls,
            "route_edges": route_edges,
            "entry_edge": route_edges[0],
            "first_seen_s": float(r.first_seen_s),
            "last_seen_s": float(r.last_seen_s),
            "cam1_raw": seg1,   # (ts, xy) en Cam 1
            "cam2_raw": seg2,   # (ts, xy) en Cam 2
            "cam1_draw": None,
            "cam2_draw": None,
            "cam2_sync_t": None,
        }
    return veh


def prepare_multicam_recon(veh: dict[str, dict], tr) -> None:
    """Ajusta los segmentos de Cam 1 y Cam 2 a la geometría real de la red SUMO."""
    H1 = load_homography(CAM1_SITE)
    H2 = load_homography(CAM2_SITE)
    db1, mw1 = load_presentation(CAM1_SITE)
    db2, mw2 = load_presentation(CAM2_SITE)

    # Tramo de Cam 2 (desde entrada de 334948975#3) para detectar el instante válido de resincronización
    try:
        cam2_entry_start_x = float(tr.lane.getShape("334948975#3_0")[0][0])
    except Exception:
        cam2_entry_start_x = 227.5

    for v in veh.values():
        edges = v["route_edges"]
        if v["status"] == "matched":
            # Segmento Cam 1 sobre [334948975#0, 334948975#2]
            path1 = _build_edge_path(tr, edges[:2])
            ts1, xy1 = v["cam1_raw"]
            v["cam1_draw"] = _snap_to_path(xy1, ts1, path1, H1, db1, mw1)

            # Segmento Cam 2 sobre [334948975#3, 334948975#5]
            path2 = _build_edge_path(tr, edges[2:])
            ts2, xy2 = v["cam2_raw"]
            draw2 = _snap_to_path(xy2, ts2, path2, H2, db2, mw2)
            v["cam2_draw"] = draw2
            # Esperar a que el vehículo en Cam 2 supere la línea de fuga y entre realmente a 334948975#3
            valid_idx = np.where(xy2[:, 0] >= cam2_entry_start_x - 10.0)[0]
            v["cam2_sync_t"] = float(ts2[int(valid_idx[0])]) if len(valid_idx) else float(ts2[0])

        elif v["status"] == "orphan_cam1" and v["cam1_raw"] is not None:
            path1 = _build_edge_path(tr, edges)
            ts1, xy1 = v["cam1_raw"]
            v["cam1_draw"] = _snap_to_path(xy1, ts1, path1, H1, db1, mw1)

        elif v["status"] == "orphan_cam2" and v["cam2_raw"] is not None:
            path2 = _build_edge_path(tr, edges)
            ts2, xy2 = v["cam2_raw"]
            v["cam2_draw"] = _snap_to_path(xy2, ts2, path2, H2, db2, mw2)


def _add_with_route(stage: SumoStage, tr, vid: str, route_edges: list[str], vtype: str) -> None:
    """Inyecta el vehículo en TraCI con la ruta completa predefinida y departPos=0.0 seguro."""
    rid = f"r_{'_'.join(route_edges)}"
    if rid not in stage._routes:
        tr.route.add(rid, route_edges)
        stage._routes.add(rid)
    try:
        tr.vehicle.add(vid, rid, typeID=vtype, departLane="0", departPos="0.0", departSpeed="0.0")
    except Exception:
        tr.vehicle.add(vid, rid, typeID=vtype, departLane="free", departPos="0.0", departSpeed="0.0")
    tr.simulationStep()
    tr.vehicle.setLaneChangeMode(vid, 0)
    tr.vehicle.setSpeedMode(vid, 0)
    try:
        tr.vehicle.subscribe(vid, _SUB_VARS)
    except Exception:
        pass


def step_multicam(stage: SumoStage, tr, veh: dict[str, dict], state: dict[str, str], t: float,
                  prov_out: dict[str, str] | None = None) -> list[str]:
    """Orquesta un instante `t` para vehículos fusionados (`matched`) y huérfanos (`orphan_cam*`).

    `state[vid]` toma valores: `"cam1"` | `"blind"` (Car-Following SUMO) | `"cam2"`.
    """
    active_ids: list[str] = []
    for vid, v in veh.items():
        status = v["status"]

        if status == "matched":
            ts1, _ = v["cam1_raw"]
            ts2, _ = v["cam2_raw"]
            t_in = float(ts1[0])
            t_blind_start = float(ts1[-1])
            t_sync_cam2 = float(v["cam2_sync_t"])
            t_out = float(ts2[-1]) + TIMEOUT_S

            if t < t_in:
                continue
            if t > t_out:
                if vid in state:
                    try:
                        stage.remove(vid)
                    except Exception:
                        pass
                    state.pop(vid, None)
                continue

            if vid not in state:
                _add_with_route(stage, tr, vid, v["route_edges"], VTYPE.get(v["cls"], "DEFAULT_VEHTYPE"))
                state[vid] = "cam1"

            if t <= t_blind_start:
                # 1) Cámara 1: control perceptual moveToXY
                if state[vid] != "cam1":
                    tr.vehicle.setSpeedMode(vid, 0)
                    state[vid] = "cam1"
                px = float(np.interp(t, ts1, v["cam1_draw"][:, 0]))
                py = float(np.interp(t, ts1, v["cam1_draw"][:, 1]))
                try:
                    tr.vehicle.moveToXY(vid, "", -1, px, py, keepRoute=1)
                    tr.vehicle.setSpeed(vid, 0.0)
                except Exception:
                    pass
                if prov_out is not None:
                    prov_out[vid] = "OBSERVED"

            elif t < t_sync_cam2:
                # 2) Punto ciego (conectores 334948975#2 -> 334948975#3): Car-Following de SUMO
                if state[vid] != "blind":
                    tr.vehicle.setSpeedMode(vid, 31)     # Car-Following + seguridad activa
                    tr.vehicle.setLaneChangeMode(vid, 0)
                    tr.vehicle.setSpeed(vid, -1.0)       # libera velocidad para cruzar a velocidad de vía (13.89 m/s)
                    state[vid] = "blind"
                if prov_out is not None:
                    prov_out[vid] = "INFERRED"

            else:
                # 3) Cámara 2: resincronización perceptual con moveToXY y salida
                if state[vid] != "cam2":
                    tr.vehicle.setSpeedMode(vid, 0)
                    state[vid] = "cam2"
                t_clamped = min(t, float(ts2[-1]))
                px = float(np.interp(t_clamped, ts2, v["cam2_draw"][:, 0]))
                py = float(np.interp(t_clamped, ts2, v["cam2_draw"][:, 1]))
                try:
                    tr.vehicle.moveToXY(vid, "", -1, px, py, keepRoute=1)
                    tr.vehicle.setSpeed(vid, 0.0)
                except Exception:
                    pass
                if prov_out is not None:
                    prov_out[vid] = "OBSERVED"

            active_ids.append(vid)

        else:
            # Huérfanos de una sola cámara (c1_* o c2_*)
            raw = v["cam1_raw"] if status == "orphan_cam1" else v["cam2_raw"]
            draw = v["cam1_draw"] if status == "orphan_cam1" else v["cam2_draw"]
            if raw is None or draw is None:
                continue
            ts, _ = raw
            t_in = float(ts[0])
            t_out = float(ts[-1]) + TIMEOUT_S
            if t < t_in:
                continue
            if t > t_out:
                if vid in state:
                    try:
                        stage.remove(vid)
                    except Exception:
                        pass
                    state.pop(vid, None)
                continue
            if vid not in state:
                _add_with_route(stage, tr, vid, v["route_edges"], VTYPE.get(v["cls"], "DEFAULT_VEHTYPE"))
                state[vid] = "local"
            t_clamped = min(t, float(ts[-1]))
            px = float(np.interp(t_clamped, ts, draw[:, 0]))
            py = float(np.interp(t_clamped, ts, draw[:, 1]))
            try:
                tr.vehicle.moveToXY(vid, "", -1, px, py, keepRoute=1)
                tr.vehicle.setSpeed(vid, 0.0)
            except Exception:
                pass
            if prov_out is not None:
                prov_out[vid] = "OBSERVED"
            active_ids.append(vid)

    return active_ids


def _setup_sumo(veh: dict[str, dict], *, gui: bool = True):
    from run_harness_viz import define_vtypes
    stage = SumoStage(NET_PATH, gui=gui, step_length=0.001).start()
    tr = stage._traci
    define_vtypes(tr)
    if gui:
        tr.gui.setSchema("View #0", "real world")
        tr.gui.setZoom("View #0", GUI_ZOOM)
        tr.gui.setOffset("View #0", GUI_OFFSET_X, GUI_OFFSET_Y)
    prepare_multicam_recon(veh, tr)
    return stage, tr


def multicam_hello_fields(merged_csv: Path = DEFAULT_MERGED_CSV) -> dict:
    """Campos `hello` del contrato v3 para el visor Web3D Multi-Cámara."""
    import hashlib
    from run_harness_viz import VTYPE, VTYPE_DEF
    site2 = load_site_config(CAM2_SITE)

    def _md5(p: Path) -> str:
        return hashlib.md5(p.read_bytes()).hexdigest()

    dims = {vc: {"length_m": VTYPE_DEF[vt][2], "width_m": VTYPE_DEF[vt][3]}
            for vc, vt in VTYPE.items()}
    return {
        "shadow": "vision",
        "mode": "replay",
        "dt": float(1 / site2.fps_ref),
        "site": "cuenca_multicam",
        "frame_geometry": None,
        "net_md5": _md5(NET_PATH),
        "artifact_md5": {"merged_csv": _md5(merged_csv)},
        "guard_mode": "assert",
        "layers": ["sumo"],
        "occupancy_semantics": {"kind": "pixel_zones", "semantics": "conteo_crudo"},
        "vclass_dims": dims,
    }


def build_multicam_view(*, seq: int, t_sim: float, drawn: dict, prov_out: dict[str, str]):
    """Construye `DecidedTickView` para el stream Web3D multi-cámara."""
    from ws_emitter.view import DecidedTickView
    sumo_subs = {
        vid: {VAR_POSITION: r[VAR_POSITION], VAR_ANGLE: r[VAR_ANGLE]}
        for vid, r in drawn.items()
        if VAR_POSITION in r and VAR_ANGLE in r
    }
    sumo_vclass = {
        vid: _VTYPE_VCLASS.get(r.get(VAR_TYPE), "car")
        for vid, r in drawn.items()
    }
    return DecidedTickView.from_vision_tick(
        seq=seq,
        t_sim=t_sim,
        sumo_subs=sumo_subs,
        sumo_prov=dict(prov_out),
        sumo_vclass=sumo_vclass,
        boxes=(),
        handoffs=(),
        occupancy_pixel={},
        guard_report={"accesses": 0, "violations": 0, "max_lookahead_s": 0.0},
    )


def run_live(veh: dict[str, dict], *, ws_em=None, gui: bool = True, play_videos: bool = True) -> None:
    site1 = load_site_config(CAM1_SITE)
    site2 = load_site_config(CAM2_SITE)
    dt = float(1 / site2.fps_ref)
    t_max = max(float(site1.t1_s), float(site2.t1_s))

    stage, tr = _setup_sumo(veh, gui=gui)
    if ws_em is not None:
        ws_em.emit_hello()

    procs = []
    if play_videos:
        procs = [
            subprocess.Popen(["ffplay", "-an", "-autoexit", "-loglevel", "quiet",
                              "-window_title", "CAM 1 (Oeste - 453731125)", "-x", "640",
                              str(site1.video_path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
            subprocess.Popen(["ffplay", "-an", "-autoexit", "-loglevel", "quiet",
                              "-window_title", "CAM 2 (Este - 577206092)", "-x", "640",
                              str(site2.video_path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        ]
        time.sleep(0.6)
    state: dict[str, str] = {}
    n_steps = int(t_max / dt)
    T0 = time.perf_counter()
    try:
        for fi in range(n_steps):
            t_v = fi * dt
            prov_out: dict[str, str] = {}
            step_multicam(stage, tr, veh, state, t_v, prov_out=prov_out)
            tr.simulationStep(t_v)
            if ws_em is not None:
                drawn = tr.vehicle.getAllSubscriptionResults()
                ws_em.emit_step(build_multicam_view(seq=fi, t_sim=t_v, drawn=drawn, prov_out=prov_out))
            target = T0 + t_v
            now = time.perf_counter()
            if now < target:
                time.sleep(target - now)
        if ws_em is not None:
            ws_em.emit_end(guard_final={"violations": 0}, status="PASS")
    finally:
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        try:
            stage.close()
        except Exception:
            pass


def run_split(veh: dict[str, dict]) -> None:
    site1 = load_site_config(CAM1_SITE)
    site2 = load_site_config(CAM2_SITE)
    dt = float(1 / site2.fps_ref)
    fps = float(site2.fps_ref)
    t_max = max(float(site1.t1_s), float(site2.t1_s))
    n_frames = int(t_max / dt)

    stage, tr = _setup_sumo(veh, gui=True)
    SHOTS.mkdir(parents=True, exist_ok=True)
    for f in SHOTS.glob("*.png"):
        f.unlink()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_mp4 = OUT_DIR / "demo_multicam.mp4"

    cap1 = cv2.VideoCapture(str(site1.video_path))
    cap2 = cv2.VideoCapture(str(site2.video_path))
    cam_w, cam_h = 640, 360
    sumo_h = cam_h * 2
    cw, chh = cam_w + SUMO_W, sumo_h + HUD_H
    ff = subprocess.Popen(["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                           "-s", f"{cw}x{chh}", "-r", f"{fps:.3f}", "-i", "-",
                           "-c:v", "libx264", "-crf", "18", "-preset", "medium",
                           "-pix_fmt", "yuv420p", str(out_mp4)], stdin=subprocess.PIPE,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    state: dict[str, str] = {}
    written = 0
    try:
        for fi in range(n_frames):
            ok1, f1 = cap1.read()
            ok2, f2 = cap2.read()
            if not (ok1 or ok2):
                break
            t = fi * dt
            active = step_multicam(stage, tr, veh, state, t)
            tr.simulationStep(t)
            shot = SHOTS / f"s_{fi % 8:d}.png"
            tr.gui.screenshot("View #0", str(shot), width=SUMO_W, height=sumo_h)
            tr.simulationStep(t + 0.001)
            sumo_img = cv2.imread(str(shot))
            if sumo_img is None:
                continue
            if sumo_img.shape[:2] != (sumo_h, SUMO_W):
                sumo_img = cv2.resize(sumo_img, (SUMO_W, sumo_h))

            canvas = np.zeros((chh, cw, 3), np.uint8)
            cv2.rectangle(canvas, (0, 0), (cw, HUD_H), (14, 14, 14), -1)
            n_g = sum(1 for k in active if k.startswith("global_"))
            cv2.putText(canvas, f"MULTI-CAMARA CUENCA | t={t:5.2f}s | activos={len(active)} (globales={n_g})",
                        (14, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 255, 160), 2, cv2.LINE_AA)
            if ok1:
                canvas[HUD_H:HUD_H + cam_h, :cam_w] = cv2.resize(f1, (cam_w, cam_h))
            if ok2:
                canvas[HUD_H + cam_h:, :cam_w] = cv2.resize(f2, (cam_w, cam_h))
            canvas[HUD_H:, cam_w:] = sumo_img
            ff.stdin.write(canvas.tobytes())
            written += 1
    finally:
        ff.stdin.close(); ff.wait(); stage.close(); cap1.release(); cap2.release()
    print(f"[ok] {out_mp4.relative_to(ROOT)} ({written} frames)")


def run_readout(veh: dict[str, dict]) -> None:
    site2 = load_site_config(CAM2_SITE)
    dt = float(1 / site2.fps_ref)
    t_end = max(v["last_seen_s"] for v in veh.values()) + TIMEOUT_S

    stage, tr = _setup_sumo(veh, gui=False)
    state: dict[str, str] = {}
    rows = []
    for fi in range(int(t_end / dt) + 2):
        t = fi * dt
        step_multicam(stage, tr, veh, state, t)
        tr.simulationStep(t)
        for vid in list(state):
            try:
                x, y = tr.vehicle.getPosition(vid)
                rows.append((vid, state[vid], round(t, 4), tr.vehicle.getRoadID(vid),
                             tr.vehicle.getLaneID(vid), round(x, 2), round(y, 2)))
            except Exception:
                state.pop(vid, None)
    stage.close()
    out = ROOT / "verify_out/events/merged/multicam_sumo_readout.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows, columns=["global_id", "phase", "t", "road", "lane", "x", "y"]).to_csv(out, index=False)
    print(f"[ok] {out.relative_to(ROOT)} ({len(rows)} muestras)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--merged-csv", type=Path, default=DEFAULT_MERGED_CSV)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--split", action="store_true")
    mode.add_argument("--readout", action="store_true")
    args = ap.parse_args()

    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    os.environ.setdefault("DISPLAY", ":1")
    veh = build_multicam_vehicles(args.merged_csv)
    print(f"[multicam] {len(veh)} vehículos cargados desde {args.merged_csv}")
    if args.readout:
        run_readout(veh)
    elif args.split:
        run_split(veh)
    else:
        run_live(veh)


if __name__ == "__main__":
    main()

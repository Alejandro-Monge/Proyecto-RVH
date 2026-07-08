"""GT2 replay driver — TraCI order=2 client.

Replays the 70 vehicles from datos/gt2/ into the multi-client SUMO (127.0.0.1:8873) that
VEINS (order=1) is reading. Fixed behavior:
  - precomputed schedule: per vehicle, resampling of the dataset (native ~33.8 ms +
    synthetic segments at 0.1 s) onto the scenario's 0.1 s grid;
  - entry at t0+delay (per-vehicle instantiation delay, from the manifest);
  - moveToXY(keepRoute=2) — exact position (kills the internal teleport);
  - setSpeed(v_real) by finite difference each tick (the messages carry getSpeed);
  - retirement by NATURAL ARRIVAL: when the schedule is exhausted the vehicle is RELEASED (setSpeed>0, no more
    moveToXY) so SUMO arrives it — NEVER vehicle.remove() (it breaks Veins'
    accounting, ASSERT TraCIScenarioManager.cc:999);
  - launched ONLY when VEINS is already in "Running simulation" (orchestration).
Deterministic: no RNG.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
# CENTER->NOSE OFFSET: the perception point is the CENTER of the
# vehicle; SUMO interprets the position as the NOSE. It is commanded as xy + (L/2)*u(heading).

sys.path.insert(0, "/usr/share/sumo/tools")
import sumolib  # noqa: E402
import traci  # noqa: E402

AQUI = Path(__file__).resolve().parent
STEP = 0.05   # fine grid (recon phase): ~20 Hz of command, close to the reference reconstruction's ~30 Hz
V_RELEASE_MIN = 3.0   # minimum speed on release (so it does not stay stuck if it ended slow)
import os as _os_p
PUERTO = int(_os_p.environ.get("TRACI_PORT", "8873"))   # parameterizable (parallel MC sweep)
# REALTIME PACING OPT-IN (view mode 5): REALITY must run at
# 1x like the shadow (the GUI's --delay is a RENDER floor, it does not hold back the
# TraCI client: measured 24.3x with --delay 50). Same pattern as the player
# (--ws-realtime): anchor (wall0, t_sim0) and ABSOLUTE targets per step — a late
# step does not accumulate debt. Pure pre-step wall-clock: the dynamics do not change (fcd
# byte-identical with/without pacing).
# Modes 2/3 do NOT export DRIVER_REALTIME: their --delay pacing stays intact.
REALTIME = _os_p.environ.get("DRIVER_REALTIME") == "1"
# ANCHOR HANDOFF: if mode 5 passes
# DRIVER_RT_ANCHOR, the anchor (wall_first, sim_first) that the player writes
# at its first emit is READ and _rt_wall0 = wall_first - sim_first is set  =>  R(w)=S(w).
# Since the shadow's content runs delay_s behind its clock (causal
# reconstruction), R(w)=S(w) makes the visible lag EXACTLY delay_s (1.0s), without the
# ~0.48s compressed by sim_first=0.4 nor the startup jitter (~0.12s). Without a
# file (or on timeout) it falls back to monotonic(): the previous behavior. Wall-clock only, does not
# touch dynamics (fcd byte-identical with/without handoff).
RT_ANCHOR_FILE = _os_p.environ.get("DRIVER_RT_ANCHOR")


def _anchor_realtime():
    if RT_ANCHOR_FILE:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            try:
                parts = Path(RT_ANCHOR_FILE).read_text().split()
                wall_first, sim_first = float(parts[0]), float(parts[1])
                anchor = wall_first - sim_first
                print(f"[driver] handoff de ancla: wall_first={wall_first:.6f} "
                      f"sim_first={sim_first:.4f} -> _rt_wall0={anchor:.6f}",
                      flush=True)
                return anchor
            except (FileNotFoundError, IndexError, ValueError):
                time.sleep(0.01)   # not yet written / half-written: retry
        print("[driver] AVISO: ancla del player no llegó en 30s; caigo a "
              "monotonic() (lag visible no garantizado a delay_s)", flush=True)
    return time.monotonic()

# vTypes with the routes_edge_start.rou.xml convention so that
# VehicleStatus.vType is comparable with those datasets: car | bus | truck.
VTYPES = {"car": dict(vClass="passenger", length=4.5),
          "bus": dict(vClass="bus", length=12.0),
          "truck": dict(vClass="truck", length=8.0)}

T_MAX = None    # --t-max <s>: cutoff for smoke test (closes cleanly; NOT the normal mode)
if "--t-max" in sys.argv:
    T_MAX = float(sys.argv[sys.argv.index("--t-max") + 1])


def _argval(name, default=None):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


# --- demo (annex video): record GT2 REALITY as a panel of shots ---
# Instead of attaching to VEINS' multi-client server, the driver can open ITS
# OWN sumo-gui (standalone, single client) to record the demo's reality.
# The operator frames it by hand (--manual-frame) and the loop captures a screenshot per
# SIM step (sh_<ms>.png), labeled by the sim clock → the compositor pairs real[T]
# with shadow[T]. No determinism gates: it is a video, not a measurement artifact.
STANDALONE_GUI = "--standalone-gui" in sys.argv
MANUAL_FRAME = "--manual-frame" in sys.argv
SHOT_DIR = _argval("--shot-dir")
GUI_SETTINGS = _argval("--gui-settings-file")
WIN_W, WIN_H = (int(v) for v in _argval("--window-size", "1148,646").split(","))
if SHOT_DIR:
    _os_p.makedirs(SHOT_DIR, exist_ok=True)
if MANUAL_FRAME and not STANDALONE_GUI:
    print("[driver] AVISO: --manual-frame requiere --standalone-gui (la ventana la "
          "abre el driver); ignoro el encuadre manual.", flush=True)
    MANUAL_FRAME = False

man = json.loads((AQUI / "datos/manifest_gt2.json").read_text())["vehiculos"]
NET = sumolib.net.readNet(str(AQUI / "red/intersection_rusia.net.xml"))


def proyeccion_inicio(entry_edge, p0):
    """(lane_idx, pos) of the trajectory point on the entry edge: the vehicle
    IS BORN where its first point is, not at the start of the edge (kills the 14-33 m flash)."""
    best = (1e9, 0, 0.0)
    for lidx, ln in enumerate(NET.getEdge(entry_edge).getLanes()):
        pts = np.asarray(ln.getShape(), float)
        arcs = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; ab = b - a
            tt = 0.0 if ab @ ab == 0 else float(np.clip((np.asarray(p0) - a) @ ab / (ab @ ab), 0, 1))
            d = float(np.hypot(*(a + tt * ab - np.asarray(p0))))
            if d < best[0]:
                best = (d, lidx, float(arcs[i] + tt * np.hypot(*ab)))
    return best[1], max(best[2], 0.1)

# --- precomputed schedule: global 0.1 s grid, positions and v_real per vehicle ---
agenda = {}
for vid, meta in man.items():
    z = np.load(AQUI / "datos/gt2" / f"{vid}.npz")
    t, xy = z["t"], z["xy"]
    dly = float(meta["delay"])
    t0, t1 = float(t[0]) + dly, float(t[-1]) + dly       # entry at t0+delay
    grid = np.round(np.arange(np.ceil(t0 / STEP) * STEP, t1, STEP), 2)
    gx = np.interp(grid - dly, t, xy[:, 0]); gy = np.interp(grid - dly, t, xy[:, 1])
    # speed and heading PRECOMPUTED by the re-render: deriving them from
    # |Δxy| per step amplified noise -> 6k "emergency braking" and heading flips.
    v = np.interp(grid - dly, t, z["speed"])
    hd = np.interp(grid - dly, t, z["heading"]) % 360.0   # heading unwrapped -> safe interp
    agenda[vid] = dict(grid=grid, x=gx, y=gy, v=v, hd=hd, meta=meta,
                       off=VTYPES.get(meta["cls"], VTYPES["car"])["length"] / 2.0,
                       estado="pendiente")  # pendiente -> activo -> soltado -> fin


# TAIL ANCHORING: the REAR part goes ON the trace (point
# of arc s0 - L/2) and the nose = tail + L*u(heading). This way the tail does not sweep by
# construction (it follows the path) and the front overhang turns on the outside, like a
# real vehicle. (Nose-on-arc anchoring left the tail cutting the curve:
# sweep 0.87-3.41 m/m on west->north.)
# STOP-LINE CLAMP: if a wait is coming at the line, the NOSE does not cross it (glide 40%/tick
# toward excess 0) and recovers smoothly on start (30% of the deficit per advance tick).
def _linea_pare(eid):
    lanes_ = NET.getEdge(eid).getLanes() if "NETX" not in globals() else NETX.getEdge(eid).getLanes()
    a_ = np.asarray(lanes_[0].getShape(), float)[-1]
    b_ = np.asarray(lanes_[-1].getShape(), float)[-1]
    if np.hypot(*(b_ - a_)) < 0.5:
        b_ = a_ + np.array([0.5, 0.0])
    return a_ - 0.8 * (b_ - a_), b_ + 0.8 * (b_ - a_)


for vid, a in agenda.items():
    pts = np.column_stack([a["x"], a["y"]])
    s0 = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    hd_r = np.radians(a["hd"])
    u_h = np.column_stack([np.sin(hd_r), np.cos(hd_r)])
    h0 = hd_r[0]; hfin = hd_r[-1]
    pre = pts[0] - 20.0 * np.array([np.sin(h0), np.cos(h0)])
    ext = pts[-1] + 20.0 * np.array([np.sin(hfin), np.cos(hfin)])
    px = np.concatenate([[pre[0]], pts[:, 0], [ext[0]]])
    py = np.concatenate([[pre[1]], pts[:, 1], [ext[1]]])
    ps = np.concatenate([[-20.0], s0, [s0[-1] + 20.0]])
    cola = np.column_stack([np.interp(s0 - a["off"], ps, px), np.interp(s0 - a["off"], ps, py)])
    frente = cola + (2.0 * a["off"]) * u_h
    # stop-line clamp on the NOSE
    la_, lb_ = _linea_pare(a["meta"]["entry_edge"])
    nrm_ = np.array([-(lb_ - la_)[1], (lb_ - la_)[0]])
    nrm_ = nrm_ / max(np.hypot(*nrm_), 1e-9)
    if (pts[-1] - la_) @ nrm_ < 0:
        nrm_ = -nrm_
    lado_ = np.sign((pts - la_) @ nrm_)
    idx_cruce = None
    for i in range(1, len(pts)):
        if lado_[i] > 0 >= lado_[i - 1]:
            idx_cruce = i; break
    if idx_cruce is not None:
        kq_ = max(int(1.0 / STEP), 1)
        quieto_ = np.zeros(len(pts), bool)
        for iq in range(len(pts)):
            i0q = max(iq - kq_, 0); i1q = min(iq + kq_, len(pts) - 1)
            quieto_[iq] = np.hypot(*(pts[i1q] - pts[i0q])) < 0.4
        s_line = float(s0[idx_cruce])
        exceso = 0.0
        for i in range(len(frente)):
            df = float((frente[i] - la_) @ nrm_)
            centro_antes = s0[i] < s_line + 3.0
            espera_delante = False
            if centro_antes and df > 0:
                j = i
                while j < len(quieto_) and s0[j] < s_line + 3.0:
                    if quieto_[j]:
                        espera_delante = True; break
                    j += 1
            if espera_delante:
                exceso = exceso + 0.7 * (df - exceso) if df > exceso else df
            else:
                exceso = max(exceso - 0.3 * max(float(s0[i] - s0[i - 1]) if i else 0.0, 0.0)
                             - (0.15 * STEP if not centro_antes else 0.0), 0.0)
            if exceso > 0:
                frente[i] = frente[i] - exceso * u_h[i]
    a["mx"] = frente[:, 0]
    a["my"] = frente[:, 1]


# CLIP-EDGE RETIREMENT: if the trajectory reaches the
# END of its exit edge, the schedule is CUT there (CUT_MARGIN before the end,
# still ON the lane) and the existing release hands the vehicle to car-following,
# which arrives it within meters: the car DISAPPEARS as it leaves the frame, just as in the video
# it leaves the field. Without this, the last moveToXY falls to the edge/outside the network, the
# natural arrival cannot operate (the vehicle is not on a lane) and it becomes a
# ZOMBIE emitting an empty laneId + invalid heading until the end of the simulation
# (it affected 11 vehicles).
# Only the contiguous FINAL BLOCK beyond the cut plane is cut: the entry
# zone may pass near the same physical point at the start of the trace (E5
# shares a corridor with -E5) and must not be cut. MARGIN 0: ONLY the
# samples after the physical end of the edge are discarded, none in frame.
CUT_MARGIN = 0.0
cortadas_tot = 0
for vid, a in agenda.items():
    sh = np.asarray(NET.getEdge(a["meta"]["exit_edge"]).getLanes()[0].getShape(), float)
    end = sh[-1]
    u = sh[-1] - sh[-2]
    u = u / max(np.hypot(*u), 1e-9)
    past = ((np.column_stack([a["mx"], a["my"]]) - end) @ u) > -CUT_MARGIN
    idx = len(past)
    while idx > 1 and past[idx - 1]:
        idx -= 1
    if idx < len(past):
        cortadas_tot += len(past) - idx
        for k in ("grid", "x", "y", "v", "hd", "mx", "my"):
            a[k] = a[k][:idx]
if cortadas_tot:
    print(f"[driver] retiro en borde: {cortadas_tot} muestras cortadas mas alla del "
          f"fin del edge de salida (margen {CUT_MARGIN} m)", flush=True)

t_fin_datos = max(a["grid"][-1] for a in agenda.values())
print(f"[driver] agenda: {len(agenda)} vehiculos, datos hasta t={t_fin_datos:.1f}s", flush=True)

if STANDALONE_GUI:
    # The driver IS the only client and opens its own PAUSED sumo-gui (no
    # --start) so the operator can frame it; the sim advances via TraCI (simulationStep),
    # like the shadow pass. No setOrder (single client, no VEINS order=1).
    _cmd = ["sumo-gui", "-n", str(AQUI / "red/intersection_rusia.net.xml"),
            "--step-length", str(STEP), "--time-to-teleport", "-1",
            "--collision.action", "none", "--ignore-route-errors",
            "--no-step-log", "true", "--end", "100000",
            "--window-size", f"{WIN_W},{WIN_H}"]
    if GUI_SETTINGS:
        _cmd += ["--gui-settings-file", GUI_SETTINGS]
    traci.start(_cmd, port=PUERTO)
    c = traci.getConnection()
    print(f"[driver] sumo-gui standalone en :{PUERTO} "
          f"(shots={'sí' if SHOT_DIR else 'no'}, encuadre-manual={MANUAL_FRAME})",
          flush=True)
else:
    c = traci.connect(port=PUERTO, host="127.0.0.1")
    c.setOrder(2)
    print("[driver] conectado, order=2", flush=True)
for name, kv in VTYPES.items():
    c.vehicletype.copy("DEFAULT_VEHTYPE", name)
    c.vehicletype.setVehicleClass(name, kv["vClass"])
    c.vehicletype.setLength(name, kv["length"])

stats = dict(insertados=0, soltados=0, err_moveto=0)
k = 0
t_ultimo_activo = None
_rt_wall0 = _anchor_realtime() if REALTIME else None   # anchor: t_sim 0 of the loop
if MANUAL_FRAME:
    # The operator frames the sumo-gui by hand (zoom/pan) and
    # presses Enter. The view stays fixed for the whole loop (simulationStep does not move it).
    # It must run in the foreground (stdin = the operator's terminal).
    print("\n[MANUAL-FRAME] Encuadra la ventana SUMO-GUI (zoom/pan) y pulsa Enter "
          "para iniciar la captura...", flush=True)
    try:
        input()
    except EOFError:
        print("[MANUAL-FRAME] stdin sin terminal interactiva; arranco sin pausa.",
              flush=True)
while True:
    tnow = round(k * STEP, 2)
    ids = set(c.vehicle.getIDList())
    for vid, a in agenda.items():
        if a["estado"] == "fin":
            continue
        g = a["grid"]
        if a["estado"] == "pendiente" and tnow >= g[0]:
            m = a["meta"]
            lidx, lpos = proyeccion_inicio(m["entry_edge"], (a["x"][0], a["y"][0]))
            c.route.add(f"r_{vid}", [m["entry_edge"]])
            c.vehicle.add(vid, f"r_{vid}", typeID=m["cls"] if m["cls"] in VTYPES else "car",
                          departLane=str(lidx), departPos=f"{lpos:.2f}", departSpeed="0")
            c.vehicle.changeTarget(vid, m["exit_edge"])
            c.vehicle.setLaneChangeMode(vid, 0)
            c.vehicle.setSpeedMode(vid, 0)
            try:
                # if SUMO accepts commanding the just-added vehicle, the first frame is already exact
                c.vehicle.moveToXY(vid, "", -1,
                                   float(a["mx"][0]),
                                   float(a["my"][0]),
                                   angle=float(a["hd"][0]), keepRoute=2)
            except traci.TraCIException:
                pass  # the projected departPos already places the birth within <1-2 m
            a["estado"] = "activo"; stats["insertados"] += 1
        elif a["estado"] == "activo":
            i = int(round((tnow - g[0]) / STEP))
            if i >= len(g):                       # schedule exhausted -> brake and RELEASE
                # STAGGERED RELEASE: handing to SUMO at data speed
                # with a queue ahead triggered emergency braking EXACTLY at the
                # handover (10 warnings, all at t==end of schedule). First it is braked
                # under command (3 m/s2) down to <=3 m/s and there it is handed to car-following.
                # DIRECT HANDOVER: the previous ramp to 3 m/s braked
                # ALL of them before handing over (the northbound one, which ends ~30 m short at ~1 m/s,
                # dragged) and did NOT avoid the handover warnings (7 with and without
                # the ramp). It is handed to car-following with whatever speed it carries.
                if vid in ids:
                    try:
                        # SNAP TO LANE on release (edge retirement):
                        # keepRoute=1 maps to the nearest position WITHIN the
                        # route -> the vehicle stays ON the exit lane and
                        # car-following can arrive it. Without this, a last data
                        # point at the clip edge leaves the vehicle off-road and
                        # it becomes a zombie (emits an empty laneId + invalid heading
                        # until the end of the simulation).
                        c.vehicle.moveToXY(vid, "", -1,
                                           float(a["mx"][-1]), float(a["my"][-1]),
                                           angle=float(a["hd"][-1]), keepRoute=1)
                        # speed floor: a vehicle stopped at the edge (tail
                        # out of frame) must still ARRIVE, not stay put
                        c.vehicle.setPreviousSpeed(vid, float(max(a["v"][-1], V_RELEASE_MIN)))
                    except traci.TraCIException:
                        pass
                    c.vehicle.setSpeedMode(vid, 31)
                    c.vehicle.setSpeed(vid, -1)
                a["estado"] = "soltado"; stats["soltados"] += 1
            elif vid in ids:                      # (the insertion tick has not appeared yet)
                try:
                    # EXPLICIT angle (smooth tangent from the re-render): SUMO no longer
                    # infers it from the noisy displacement between steps
                    c.vehicle.moveToXY(vid, "", -1,
                                   float(a["mx"][i]),
                                   float(a["my"][i]),
                                   angle=float(a["hd"][i]), keepRoute=2)
                    c.vehicle.setSpeed(vid, float(a["v"][i]))
                except traci.TraCIException:
                    stats["err_moveto"] += 1
        elif a["estado"] == "soltado" and vid not in ids:
            a["estado"] = "fin"                   # SUMO arrived it (VEINS sees it as arrived)
    k += 1
    t_next = round(k * STEP, 2)
    if REALTIME:
        _tgt = _rt_wall0 + t_next          # ABSOLUTE target against the anchor
        _ahora = time.monotonic()
        if _ahora < _tgt:
            time.sleep(_tgt - _ahora)
    try:
        c.simulationStep(t_next)
    except traci.FatalTraCIError:
        print(f"[driver] sesion terminada por el servidor en t={tnow}", flush=True)
        break
    # PASSIVE screenshot of reality, labeled by SIM (t_next). The
    # gui.screenshot only reads/renders (does not touch dynamics). Deferred: the write is
    # offloaded by the redraw of the loop's next simulationStep (paused mode).
    if SHOT_DIR:
        try:
            c.gui.screenshot(
                "View #0",
                str(Path(SHOT_DIR) / f"sh_{int(round(t_next * 1000)):09d}.png"),
                width=WIN_W, height=WIN_H)
        except traci.TraCIException:
            pass
    if T_MAX is not None and abs(tnow - round(T_MAX * 0.6, 1)) < 1e-9:
        # smoke evidence: sumo-gui saves ITS OWN view (only if the server is GUI)
        try:
            c.gui.screenshot("View #0", str(AQUI / "datos/_smoke_vista.png"))
            print(f"[driver] screenshot de la vista pedido en t={tnow}", flush=True)
        except traci.TraCIException:
            pass
    if T_MAX is not None and tnow >= T_MAX:
        print(f"[driver] --t-max {T_MAX}: corte de smoke en t={tnow}", flush=True)
        break
    vivos = [v for v, a in agenda.items() if a["estado"] in ("activo", "soltado")]
    if tnow > t_fin_datos and not vivos:
        print(f"[driver] todos arribados en t={tnow:.1f}s", flush=True)
        break
    if tnow > t_fin_datos + 60:
        print(f"[driver] TIMEOUT de arribo; aun vivos: {vivos}", flush=True)
        break
    if k % 300 == 0:
        drift = (f" drift={((time.monotonic() - _rt_wall0) - tnow):+.4f}s"
                 if REALTIME else "")
        print(f"[driver] t={tnow:5.1f}s activos={sum(1 for a in agenda.values() if a['estado']=='activo'):2d} "
              f"soltados={sum(1 for a in agenda.values() if a['estado']=='soltado')} "
              f"fin={sum(1 for a in agenda.values() if a['estado']=='fin')}{drift}", flush=True)

try:
    c.close()
except Exception:  # noqa: BLE001
    pass
fin = sum(1 for a in agenda.values() if a["estado"] == "fin")
print(f"[driver] FIN: insertados={stats['insertados']}/{len(agenda)} arribados={fin}/{len(agenda)} "
      f"err_moveto={stats['err_moveto']}", flush=True)

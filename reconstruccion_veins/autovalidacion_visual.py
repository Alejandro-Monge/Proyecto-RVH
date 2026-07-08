"""VISUAL AUTO-VALIDATION of GT2 mobility.

Mobility-only replay in sumo-gui (HOST, a single client, no VEINS) that captures the
view at KEY INSTANTS with focus on the historically problematic vehicles. The
captures are reviewed to visually validate the reconstructed mobility.

Instants/foci (REPLAY time = t_video + vehicle delay):
  giro_v0        t≈12   west→north bus mid-curve (net stub)
  cola_sur       t≈18   straight-through vehicles and the south queue (v7/v8)
  v12_espera     t≈30   v12 stopped inside the crossing (most synthesis)
  cola_norte     t≈48   long north queue (v1/v2, 45-55 s waits)
  giros_sur_este t≈65   v10/v42 turning + merges (wide view of the crossing)
  norte_este     t≈88   v34 turning north→east
  recta_oeste    t≈145  west→east buses on the straight (v95)
  cola_este      t≈152  east→west queue (v44/v45)
  giro_oeste_sur t≈19.5 v18 turning west→south
  S_sur_oeste/v72 t≈58/86  v9/v72 south→west
  S_este_sur     t≈156.5  v109 entering the east→south turn (the S window)
Coverage: the FOUR turn types + the windows where "opening up before
turning" was detected (blind spot of the original 8 shots).

Regression criteria (what is reviewed in each PNG):
  1) straight-through vehicles and queues: bodies WITHIN the lane, aligned with their axis (no
     skewed or displaced cars); queues with a visible gap between bodies, no overlap;
  2) turns: continuous curve through the interior of the crossing, no corners or jumps;
  3) nothing off the roadway; nobody pointing against their direction of travel;
  4) v12: waits still and oriented; its exit to the north is continuous.

Usage: ../.venv/bin/python autovalidacion_visual.py  ->  datos/autoval/<label>.png
"""
import json
import shutil
import sys
from pathlib import Path

import numpy as np
# CENTER->NOSE OFFSET: the perception point is the CENTER of the
# vehicle; SUMO interprets the position as the NOSE. It is commanded as xy + (L/2)*u(heading).

sys.path.insert(0, "/usr/share/sumo/tools")
import sumolib  # noqa: E402
import traci  # noqa: E402

AQUI = Path(__file__).resolve().parent
OUT = AQUI / "datos/autoval"
STEP = 0.05
VTYPES = {"car": dict(vClass="passenger", length=4.5),
          "bus": dict(vClass="bus", length=12.0),
          "truck": dict(vClass="truck", length=8.0)}
# (t_replay, label, focus_vehicle | None=crossing, zoom)
TOMAS = [(12.0, "giro_v0", "v0", 1400), (18.0, "cola_sur", "v8", 900),
         (19.5, "giro_oeste_sur", "v18", 1100), (30.0, "v12_espera", "v12", 1400),
         (48.0, "cola_norte", "v2", 900), (58.0, "S_sur_oeste_v9", "v9", 1100),
         (65.0, "giros_sur_este", None, 800), (86.0, "sur_oeste_v72", "v72", 1100),
         (88.0, "norte_este", "v34", 1200), (145.0, "recta_oeste", "v95", 900),
         (152.0, "cola_este", "v45", 900), (161.0, "S_este_sur_v109", "v109", 1100)]

if OUT.exists():
    shutil.rmtree(OUT)
OUT.mkdir(parents=True)
man = json.loads((AQUI / "datos/manifest_gt2.json").read_text())["vehiculos"]
NETX = sumolib.net.readNet(str(AQUI / "red/intersection_rusia.net.xml"))


def proyeccion_inicio(entry_edge, p0):
    best = (1e9, 0, 0.0)
    for lidx, ln in enumerate(NETX.getEdge(entry_edge).getLanes()):
        pts = np.asarray(ln.getShape(), float)
        arcs = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; ab = b - a
            tt = 0.0 if ab @ ab == 0 else float(np.clip((np.asarray(p0) - a) @ ab / (ab @ ab), 0, 1))
            d = float(np.hypot(*(a + tt * ab - np.asarray(p0))))
            if d < best[0]:
                best = (d, lidx, float(arcs[i] + tt * np.hypot(*ab)))
    return best[1], max(best[2], 0.1)

agenda = {}
for vid, meta in man.items():
    z = np.load(AQUI / "datos/gt2" / f"{vid}.npz")
    t = z["t"]; dly = float(meta["delay"])
    grid = np.round(np.arange(np.ceil((t[0] + dly) / STEP) * STEP, t[-1] + dly, STEP), 2)
    agenda[vid] = dict(grid=grid,
                       x=np.interp(grid - dly, t, z["xy"][:, 0]),
                       y=np.interp(grid - dly, t, z["xy"][:, 1]),
                       v=np.interp(grid - dly, t, z["speed"]),
                       hd=np.interp(grid - dly, t, z["heading"]) % 360.0,
                       off=VTYPES.get(meta["cls"], VTYPES["car"])["length"] / 2.0,
                       meta=meta, estado="pendiente")


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



traci.start(["sumo-gui", "-n", str(AQUI / "red/intersection_rusia.net.xml"),
             "--step-length", str(STEP), "--tls.all-off", "true",
             "--collision.action", "none", "--time-to-teleport", "-1",
             "--ignore-route-errors", "true", "--no-warnings", "true",
             "--window-size", "1500,1000", "--start", "--delay", "0"])
for name, kv in VTYPES.items():
    traci.vehicletype.copy("DEFAULT_VEHTYPE", name)
    traci.vehicletype.setVehicleClass(name, kv["vClass"])
    traci.vehicletype.setLength(name, kv["length"])
try:
    traci.gui.setSchema("View #0", "real world")
except traci.TraCIException:
    pass
J5 = (57.01, 74.40)

tomas = sorted(TOMAS)
pend = list(tomas)
t_fin = max(a["grid"][-1] for a in agenda.values())
k = 0
while k * STEP <= min(t_fin, pend[-1][0] + 1.0 if pend else t_fin):
    tnow = round(k * STEP, 2)
    ids = set(traci.vehicle.getIDList())
    for vid, a in agenda.items():
        g = a["grid"]; m = a["meta"]
        if a["estado"] == "pendiente" and tnow >= g[0]:
            traci.route.add(f"r_{vid}", [m["entry_edge"]])
            lidx, lpos = proyeccion_inicio(m["entry_edge"], (a["x"][0], a["y"][0]))
            traci.vehicle.add(vid, f"r_{vid}", typeID=m["cls"] if m["cls"] in VTYPES else "car",
                              departLane=str(lidx), departPos=f"{lpos:.2f}", departSpeed="0")
            traci.vehicle.changeTarget(vid, m["exit_edge"])
            traci.vehicle.setLaneChangeMode(vid, 0); traci.vehicle.setSpeedMode(vid, 0)
            try:
                traci.vehicle.moveToXY(vid, "", -1,
                                   float(a["mx"][0]),
                                   float(a["my"][0]),
                                   angle=float(a["hd"][0]), keepRoute=2)
            except traci.TraCIException:
                pass
            a["estado"] = "activo"
        elif a["estado"] == "activo":
            i = int(round((tnow - g[0]) / STEP))
            if i >= len(g):
                try: traci.vehicle.remove(vid)
                except Exception: pass
                a["estado"] = "fin"
            elif vid in ids:
                traci.vehicle.moveToXY(vid, "", -1,
                                   float(a["mx"][i]),
                                   float(a["my"][i]),
                                   angle=float(a["hd"][i]), keepRoute=2)
                traci.vehicle.setSpeed(vid, float(a["v"][i]))
    traci.simulationStep(round((k + 1) * STEP, 2))
    if pend and tnow >= pend[0][0] - 0.2 and tnow < pend[0][0]:
        _, _, foco, zoom = pend[0]
        try:
            # DETERMINISTIC framing: setOffset to the current position of the target
            # (trackVehicle turned out unreliable after a previous track)
            if foco and foco in traci.vehicle.getIDList():
                px, py = traci.vehicle.getPosition(foco)
                traci.gui.setOffset("View #0", px, py)
            else:
                traci.gui.setOffset("View #0", *J5)
            traci.gui.setZoom("View #0", zoom)
        except traci.TraCIException:
            pass
    if pend and tnow >= pend[0][0]:
        tk, label, foco, zoom = pend.pop(0)
        traci.gui.screenshot("View #0", str(OUT / f"{label}_t{int(tk)}.png"))
        print(f"[autoval] captura {label} en t={tnow}", flush=True)
    k += 1
traci.close()
print(f"[autoval] {len(tomas) - len(pend)}/{len(tomas)} capturas en {OUT}")

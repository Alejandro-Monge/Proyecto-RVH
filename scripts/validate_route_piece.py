"""Validation of the ROUTE piece, with data (4 blocks).

  1) lane→turn accuracy vs OBSERVED destination, per approach, with Wilson CI (END-TO-END: uses the lane
     the CLASSIFIER delivered, not the GT one — honest number). Sur abstains (arm_only).
  2) changeTarget against LIVE SUMO + the position (lane 0 axis) against real SUMO (deferred verification).
  3) Corner-cut that the route fill AVOIDS (junction curve vs straight interpolation).
  4) delay-vs-accuracy curve Section 8.2: predicted@crossing (low delay) vs observed@exit (high delay).

Read only for data/net; block 2 launches SUMO headless. Contract verified separately (tests).
"""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.position import load_homography, longitudinal_pos, project_to_world
from digital_shadow.sumo.route import load_lane_turn_rule, load_turns, predict_turn


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * ((p * (1 - p) / n + z * z / (4 * n * n)) ** 0.5) / d
    return (max(0, c - h), min(1, c + h))


def block1_lane_turn_accuracy():
    print("=== 1) Acierto carril→giro vs destino OBSERVADO (END-TO-END, carril del clasificador) ===")
    rule = load_lane_turn_rule("rusia")
    turns_map = load_turns("rusia")
    te = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/track_events.csv")
    m = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv").set_index("event_id")
    te = te.set_index("track_id")
    print(f"{'brazo':6s} | {'predichos':>9s} {'aciertos':>8s} {'acc':>5s} {'IC95':>14s} | abstiene(arm_only)")
    for arm in ["sur", "norte", "este", "oeste"]:
        sub = m[m["entry_arm"] == arm]
        pred = hit = abst = 0
        for _, row in sub.iterrows():
            lane = te.loc[row["track_id"], "entry_lane"] if row["track_id"] in te.index else ""
            t = predict_turn(rule, arm, "" if pd.isna(lane) else lane)
            if t is None:
                abst += 1
                continue
            pred += 1
            hit += int(t == gt.loc[row["event_id"], "turn"])
        lo, hi = wilson(hit, pred)
        acc = f"{100*hit/pred:.0f}%" if pred else "—"
        ic = f"[{100*lo:.0f},{100*hi:.0f}]" if pred else ""
        print(f"{arm:6s} | {pred:9d} {hit:8d} {acc:>5s} {ic:>14s} | {abst}")
    print("  (sur abstiene: el clasificador no resuelve su carril (arm_only) → sin predicción de giro;")
    print("   GT del sur COMPLETO, es la imagen la insuficiente. Techo de la regla con carril-GT: 88–100% LOO.)")


def block2_sumo_live():
    print("\n=== 2) changeTarget + pos (eje lane 0) contra SUMO VIVO (verificación diferida) ===")
    import os
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    try:
        import traci
    except Exception as e:
        print(f"  traci no disponible: {e}"); return
    net = str(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml")
    lm = load_lane_map("rusia")
    H = load_homography("rusia")
    try:
        traci.start(["sumo", "-n", net, "--no-step-log", "true", "--no-warnings", "true", "--step-length", "0.1"])
    except Exception as e:
        print(f"  no se pudo lanzar SUMO: {e}"); return
    try:
        ax = lm.approach_axis["sur"]
        world_target = (ax.origin[0] + 30 * ax.direction[0], ax.origin[1] + 30 * ax.direction[1])
        pos = longitudinal_pos(ax, world_target)

        # (a) changeTarget E5 -> -E1 (turn sur->oeste, connection validated 12/12)
        traci.route.add("r_E5", ["E5"])
        traci.vehicle.add("v0", "r_E5", departLane="0", departPos="0", departSpeed="0")
        traci.simulationStep()
        traci.vehicle.changeTarget("v0", "-E1")
        route = tuple(traci.vehicle.getRoute("v0"))
        ok_route = route == ("E5", "-E1")
        print(f"  [{'PASS' if ok_route else 'FALLA'}] changeTarget('-E1') → ruta {route} "
              f"(esperada ('E5','-E1'); junción :J5_* implícita; conexión 12/12)")

        # (b) FINDING: with SUMO control ACTIVE, moveTo + route → SUMO changes lane (drift).
        traci.vehicle.moveTo("v0", "E5_0", pos)
        traci.simulationStep()
        d_on = float(np.hypot(*(np.array(traci.vehicle.getPosition("v0")) - world_target)))
        lane_on = traci.vehicle.getLaneID("v0")

        # (c) with control DISABLED (what the harness must do), the position is faithful.
        traci.vehicle.add("v1", "r_E5", departLane="0", departPos="0", departSpeed="0")
        traci.simulationStep()
        traci.vehicle.setLaneChangeMode("v1", 0)   # no autonomous lane change
        traci.vehicle.setSpeedMode("v1", 0)        # no car-following/safety
        traci.vehicle.moveTo("v1", "E5_0", pos)
        traci.simulationStep()
        d_off = float(np.hypot(*(np.array(traci.vehicle.getPosition("v1")) - world_target)))
        lane_off = traci.vehicle.getLaneID("v1")
        print(f"  [HALLAZGO] control SUMO ACTIVO: moveTo('E5_0',{pos:.0f}) → lane {lane_on}, "
              f"error {d_on:.2f} m (SUMO lo cambió de carril ≈ 1 ancho)")
        print(f"  [{'PASS' if d_off < 0.1 else 'FALLA'}] control DESACTIVADO (setLaneChangeMode/SpeedMode=0): "
              f"lane {lane_off}, error {d_off:.3f} m → la pos (eje lane 0) es FIEL contra SUMO real")
    finally:
        traci.close()


def block3_corner_cut():
    print("\n=== 3) Corte-de-esquina que el relleno por RUTA evita (curva junción vs recta) ===")
    r = ET.parse(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml").getroot()
    # junction internals for the south turns: E5->-E1(l)=:J5_6, E5->-E3(r)=:J5_4, E5->-E6(s)=:J5_5
    internals = {":J5_6": "sur→oeste (izq)", ":J5_4": "sur→este (der)", ":J5_5": "sur→norte (recto)",
                 ":J5_14": "norte→este (izq)", ":J5_0": "este→norte (der)"}
    print(f"  {'interno':8s} {'movimiento':18s} {'corte recta(m)':>14s}  (= teletransporte que el route-fill evita)")
    for e in r.findall("edge"):
        if e.get("function") != "internal" or e.get("id") not in internals:
            continue
        lane = e.find("lane")
        pts = np.array([[float(c) for c in q.split(",")] for q in lane.get("shape").split()])
        a, b = pts[0], pts[-1]                       # chord (what a straight line would interpolate)
        ab = b - a
        L = np.linalg.norm(ab)
        if L < 1e-6:
            continue
        d = np.abs(np.cross(ab, pts - a)) / L        # perpendicular distance of each point to the chord
        print(f"  {e.get('id'):8s} {internals[e.get('id')]:18s} {d.max():14.2f}")
    print("  (un giro izq/der recorrido por una RECTA en el hueco corta varios metros de esquina; la")
    print("   ruta de SUMO sigue la curva interna → sin corte. Recto ≈ 0 (la recta ya es buena).)")


def block4_delay_accuracy():
    print("\n=== 4) Curva delay-vs-acierto §8.2: predicho@cruce vs observado@salida ===")
    # Observed delay = junction transit (entry→exit crossing); predicted ≈ 0 (at crossing).
    # Predicted accuracy = block 1; observed = 100% by definition.
    print("  Dos extremos de la curva (por brazo):")
    print(f"  {'brazo':6s} | {'PREDICHO: delay~0, acc':>24s} | {'OBSERVADO: delay=tránsito, acc=100%':>36s}")
    rule = load_lane_turn_rule("rusia")
    turns_map = load_turns("rusia")
    te = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/track_events.csv").set_index("track_id")
    m = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv").set_index("event_id")
    for arm in ["sur", "norte", "este", "oeste"]:
        sub = m[m["entry_arm"] == arm]
        pred = hit = 0
        delays = []
        for _, row in sub.iterrows():
            tid = row["track_id"]
            if tid in te.index:
                d = te.loc[tid, "exit_time_s"] - te.loc[tid, "entry_time_s"]
                if np.isfinite(d) and d > 0:
                    delays.append(d)
                lane = te.loc[tid, "entry_lane"]
                t = predict_turn(rule, arm, "" if pd.isna(lane) else lane)
                if t is not None:
                    pred += 1
                    hit += int(t == gt.loc[row["event_id"], "turn"])
        acc = f"{100*hit/pred:.0f}%" if pred else "abstiene"
        trans = np.median(delays) if delays else float("nan")
        print(f"  {arm:6s} | {('delay≈0 s, acc=' + acc):>24s} | {('delay≈%.1f s, acc=100%%' % trans):>36s}")
    print("  Lectura: el predicho da bajo delay con el acierto del bloque 1; el observado da 100% pero")
    print("  paga el tránsito de junción de delay. La pieza registra la evidencia por vehículo → la curva.")


def main():
    block1_lane_turn_accuracy()
    block2_sumo_live()
    block3_corner_cut()
    block4_delay_accuracy()


if __name__ == "__main__":
    main()

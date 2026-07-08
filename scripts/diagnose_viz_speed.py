"""DIAGNOSTIC: why does the viz run slower than real time if Section 7.6 (compute) has slack (8.78 ms)?

Measures the viz loop PER STAGE, live, and separates two distinct claims:
  (a) the COMPUTE PIPELINE runs in real time (the paper metric, Section 7.6) — headless, without GUI.
  (b) the synchronized VISUALIZATION runs at real speed (demo convenience, not a paper metric).

Per-frame breakdown: SUMO command (moveToXY+sync), GUI render (simulationStep with sumo-gui),
capture (screenshot+disk), video decoding (cv2 seek+read), composition (cv2+imwrite).
Variants: FULL, WITHOUT capture, WITHOUT video, and the headless step (sumo) vs gui. It does NOT assume the bottleneck: it measures it.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from run_harness_viz import COCO_NAME, SHOTS, compose, define_vtypes, run_core, VTYPE  # noqa

DT = 12 / 355
NET = ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml"
VIDEO = ROOT / "data/sources/rusia/rusia_tramo_000-180.mp4"


def step_cost(gui: bool, n=400):
    """Per-frame cost of the SUMO COMMAND (moveToXY+simulationStep): gui vs headless. Isolates the render."""
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo"); os.environ.setdefault("DISPLAY", ":1")
    from digital_shadow.sumo.stage import SumoStage
    st = SumoStage(NET, gui=gui, step_length=0.001).start()
    if gui:
        st._traci.gui.setOffset("View #0", 57.0, 74.4); st._traci.gui.setZoom("View #0", 700)
    st.add("v", "E5", 0, 5.0); p = st._traci.lane.getShape("E5_0")[0]
    ts = []
    for i in range(n):
        a = time.perf_counter()
        st.place_xy("v", p[0], p[1]); st.sync((i + 1) * 0.034)
        ts.append((time.perf_counter() - a) * 1000)
    st.close()
    return float(np.median(ts[50:]))


def main():
    print("Corriendo el núcleo causal (compute, una vez)...")
    t0 = time.perf_counter(); veh, H = run_core(); t_core = time.perf_counter() - t0
    print(f"  núcleo causal (todo el clip, 5325 frames): {t_core:.1f}s = {1000*t_core/5325:.3f} ms/frame\n")

    # --- isolate the cost of the SUMO COMMAND: headless vs gui ---
    print("=== Coste del comando SUMO (moveToXY+simulationStep), aislado ===")
    ms_head = step_cost(gui=False)
    ms_gui = step_cost(gui=True)
    print(f"  headless (sumo):   {ms_head:6.3f} ms/frame   <- es lo que mide el §7.6 (pipeline de cómputo)")
    print(f"  con GUI (sumo-gui):{ms_gui:6.3f} ms/frame   <- el simulationStep RE-RENDERIZA la GUI")
    print(f"  -> el render de la GUI añade {ms_gui-ms_head:.2f} ms/frame al step (×{ms_gui/max(ms_head,1e-3):.0f}).\n")

    # --- viz loop by stages (FULL) ---
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo"); os.environ.setdefault("DISPLAY", ":1")
    from digital_shadow.sumo.stage import SumoStage
    SHOTS.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(VIDEO))
    stage = SumoStage(NET, gui=True, step_length=0.001).start(); tr = stage._traci
    define_vtypes(tr); tr.gui.setSchema("View #0", "real world")
    tr.gui.setZoom("View #0", 700); tr.gui.setOffset("View #0", 57.0, 74.4)
    live = {}
    T = {k: [] for k in ["traci", "step", "shot", "video", "compose"]}
    f0, f1, STEP = int(45 / DT), int(60 / DT), 3
    nshot = 0
    for fi in range(f0, f1, STEP):
        t = fi * DT
        a = time.perf_counter()
        for vid, v in veh.items():
            tshift = t - v["delay"]
            if v["t"][0] <= tshift <= v["t"][-1]:
                x = float(np.interp(tshift, v["t"], v["xy"][:, 0])); y = float(np.interp(tshift, v["t"], v["xy"][:, 1]))
                if vid not in live and v["entry_edge"]:
                    try:
                        stage.add(vid, v["entry_edge"], 0, 5.0, vtype=VTYPE.get(v["cls"], "DEFAULT_VEHTYPE"))
                        live[vid] = True
                    except Exception:
                        live[vid] = "gone"      # already existed / arrived: do not retry
                if live.get(vid) is True:
                    try: tr.vehicle.moveToXY(vid, "", -1, x, y, keepRoute=1)
                    except Exception: live[vid] = "gone"
            elif live.get(vid) is True and tshift > v["t"][-1]:
                try: stage.remove(vid)
                except Exception: pass
                live[vid] = "gone"
        b = time.perf_counter(); tr.simulationStep(t); c = time.perf_counter()
        shot = SHOTS / f"d_{nshot:04d}.png"; tr.gui.screenshot("View #0", str(shot)); tr.simulationStep(t + 0.001)
        d = time.perf_counter()
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi); ok, _vf = cap.read(); e = time.perf_counter()
        compose(cap, fi, shot, t, [0.4], SHOTS / f"dc_{nshot:04d}.png"); f = time.perf_counter()
        T["traci"].append((b - a) * 1000); T["step"].append((c - b) * 1000)
        T["shot"].append((d - c) * 1000); T["video"].append((e - d) * 1000); T["compose"].append((f - e) * 1000)
        nshot += 1
    stage.close(); cap.release()

    print("=== Bucle de viz EN VIVO, desglose por etapa (mediana ms/frame) ===")
    order = [("traci", "comando SUMO (moveToXY)"), ("step", "render GUI (simulationStep)"),
             ("shot", "captura (screenshot+disco)"), ("video", "decodificar video (cv2 seek+read)"),
             ("compose", "componer paneles (cv2+imwrite)")]
    total = 0.0
    for k, label in order:
        m = float(np.median(T[k])); total += m
        print(f"  {label:38s} {m:8.2f}")
    print(f"  {'TOTAL bucle de viz':38s} {total:8.2f}  vs tiempo real {1000*DT:.1f} ms/frame")
    budget = 1000 * DT
    print(f"  -> {'MÁS LENTO que tiempo real' if total > budget else 'tiempo real'} "
          f"(×{total/budget:.1f} el presupuesto).")
    cuello = max(order, key=lambda kl: np.median(T[kl[0]]))
    print(f"\n  CUELLO (medido): '{cuello[1]}' = {np.median(T[cuello[0]]):.1f} ms/frame.")
    print(f"  Comparar: el COMANDO SUMO sin GUI (pipeline §7.6) = {ms_head:.2f} ms; con GUI = {ms_gui:.2f} ms.")
    print(f"  (Interpretación a/b según qué etapa sea el cuello: render/captura/video → (b) viz;")
    print(f"   el comando de cómputo en sí → (a) pipeline. Los datos deciden.)")


if __name__ == "__main__":
    main()

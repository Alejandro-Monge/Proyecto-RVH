"""SumoStage smoke against LIVE SUMO (Part 1 of the harness). 4 checks:
  1) clock sync (absolute re-pointing, no drift).
  2) keepRoute for the junction: which moveToXY bit places at exact (x,y) over the internal lane?
  3) gap fill: advancing along the :J5_6_0 shape (sur->oeste) follows the CURVE, not the straight line.
  4) life cycle: add -> place_lane(approach) -> place_xy(junction) -> place_lane(egress) -> remove.
Read only; launches SUMO headless.
"""
from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.stage import GapFiller, SumoStage, point_at_arclen, polyline_arclen

NET = ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml"


def main():
    lm = load_lane_map("rusia")
    stage = SumoStage(NET, gui=False, step_length=0.001, lane_map=lm).start()
    try:
        # 1) clock sync
        errs = [abs(stage.sync(float(f * Fraction(12, 355))) - float(f * Fraction(12, 355)))
                for f in range(1, 200)]
        print(f"1) sync reloj: |err| máx por frame = {max(errs)*1000:.2f} ms "
              f"({'PASS' if max(errs) < 0.002 else 'FALLA'}; re-apuntado absoluto, sin deriva)")

        # 2) keepRoute: internal shape sur->oeste, midpoint; test bits
        shape = stage.internal_shape("sur", "oeste")          # from traci.lane.getShape(:J5_6_0)
        L = polyline_arclen(shape)
        target = point_at_arclen(shape, L[-1] / 2)            # midpoint of the curve
        print(f"2) keepRoute (junción) — objetivo {tuple(round(x,2) for x in target)}:")
        for kr in (2, 1):
            stage.add(f"k{kr}", "E5", 0, 0.0)
            stage.set_target(f"k{kr}", "-E1")
            stage.place_xy(f"k{kr}", target[0], target[1], keep_route=kr)
            stage.sync(stage._traci.simulation.getTime() + 0.001)
            got = stage.position(f"k{kr}")
            err = float(np.hypot(got[0] - target[0], got[1] - target[1]))
            print(f"   keepRoute={kr}: SUMO en {tuple(round(x,2) for x in got)}, error {err:.3f} m, "
                  f"lane {stage.lane_of(f'k{kr}')}  {'<- exacto' if err < 0.1 else ''}")
            stage.remove(f"k{kr}")

        # 3) gap fill along the curve vs straight
        gf = GapFiller(shape, speed=8.0, t0=0.0)              # 8 m/s over :J5_6_0
        ts = np.linspace(0, gf.length / 8.0, 20)
        path = np.array([gf.at(t) for t in ts])
        a, b = path[0], path[-1]
        chord = np.abs(np.cross(b - a, path - a)) / np.hypot(*(b - a))
        print(f"3) relleno por curva: máx desviación de la recta = {chord.max():.2f} m "
              f"({'PASS' if chord.max() > 3 else 'FALLA'}; sigue la curva, no corta la esquina)")

        # 4) full life cycle
        stage.add("life", "E5", 2, 5.0)                       # instantiate in the south approach, lane 3
        present = "life" in stage.vehicles
        stage.place_lane("life", "E5_2", 40.0)               # move in the approach
        stage.sync(stage._traci.simulation.getTime() + 0.001)
        mid = point_at_arclen(shape, L[-1] / 2)
        stage.place_xy("life", mid[0], mid[1], keep_route=2)  # move through the junction
        stage.sync(stage._traci.simulation.getTime() + 0.001)
        stage.place_lane("life", "-E1_1", 10.0)              # move in the egress
        stage.sync(stage._traci.simulation.getTime() + 0.001)
        stage.remove("life")
        stage.sync(stage._traci.simulation.getTime() + 0.001)
        absent = "life" not in stage.vehicles
        print(f"4) ciclo de vida: instanciado={present}, retirado={absent} "
              f"({'PASS' if present and absent else 'FALLA'}; add->approach->junción->egreso->remove)")
    finally:
        stage.close()


if __name__ == "__main__":
    main()

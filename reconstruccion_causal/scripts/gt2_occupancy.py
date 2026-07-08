"""gt2_occupancy — occupancy GT from the fcd of the layer-0 replica (OFFLINE).

Takes the fcd-output of the REALITY run (Via B chain:
canonical cfg rusia_via_b.sumo.cfg + driver_replay, mobility GT2 f712b746,
62 vehicles) and applies THE SAME rule as the live hook (occupancy_core,
direct import: a single implementation). Independent of the messages channel
by construction: it does not read the RSU CSV nor anything from the shadow.

The fcd carries lane and speed directly in each <vehicle> of the <timestep>
(default fcd-output attributes, verified against a real project fcd,
SUMO 1.12: id,x,y,angle,type,speed,pos,lane,slope). The .rou is NOT an
input: in Via B it does not exist (the demand is injected by the driver over
TraCI) and everything needed comes in the fcd itself.

Grid: the layer-0 SUMO runs at 0.05 s; the occupancy series goes on the
shadow's LOGICAL tick (0.1 s), so only the timesteps that are multiples of
--tick are sampled. Output: SAME schema as the live hook —
  sim_time_s, brazo, carril, ocupacion, detenidos
(one row per lane and tick, explicit zeros).

Usage:
  python3 scripts/gt2_occupancy.py --fcd inputs/fcd_capa0_gt2.xml \
      --config ../config/sites/rusia/occupancy.yaml --out <gt_ocupacion.csv>
"""
import argparse
import csv
import gzip
import hashlib
import os
import xml.etree.ElementTree as ET

import yaml

import occupancy_core as occ_core


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fcd", required=True,
                    help="fcd-output de la capa-0 (.xml o .xml.gz)")
    ap.add_argument("--config", required=True,
                    help="config/sites/rusia/occupancy.yaml")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tick", type=float, default=0.1,
                    help="rejilla lógica de salida (s); se muestrean solo los "
                         "timesteps del fcd múltiplos de este valor")
    args = ap.parse_args()

    cfg_path = os.path.abspath(args.config)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(cfg_path),
                                             "..", "..", ".."))
    lane_map_path = os.path.join(repo_root, cfg["lane_map_ref"])
    with open(lane_map_path) as f:
        approach_lanes = occ_core.load_approach_lanes(yaml.safe_load(f))
    halt = float(cfg["halting_speed_mps"])

    opener = gzip.open if args.fcd.endswith(".gz") else open
    n_rows = n_ticks = n_saltados = 0
    vehs = set()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with opener(args.fcd, "rb") as fin, open(args.out, "w", newline="") as fout:
        w = csv.writer(fout)
        w.writerow(["sim_time_s", "brazo", "carril", "ocupacion", "detenidos"])
        # iterparse with pruning: the whole tree is not loaded into memory
        for _ev, ts in ET.iterparse(fin):
            if ts.tag != "timestep":
                continue
            t = float(ts.get("time"))
            if abs(t / args.tick - round(t / args.tick)) > 1e-6:
                n_saltados += 1   # fcd sub-step outside the logical grid
                ts.clear()
                continue
            states = []
            for v in ts.iter("vehicle"):
                vehs.add(v.get("id"))
                states.append((v.get("id"), v.get("lane", ""),
                               float(v.get("speed", 0.0))))
            for (brazo, carril), cnt in sorted(occ_core.occupancy_counts(
                    states, approach_lanes, halt).items()):
                w.writerow([f"{t:.1f}", brazo, carril,
                            cnt["ocupacion"], cnt["detenidos"]])
                n_rows += 1
            n_ticks += 1
            ts.clear()

    with open(lane_map_path, "rb") as f:
        lm_md5 = hashlib.md5(f.read()).hexdigest()
    print(f"[gt2_occupancy] ticks={n_ticks} (sub-steps saltados={n_saltados}) "
          f"vehiculos={len(vehs)} filas={n_rows} halting<{halt} "
          f"lane_map_md5={lm_md5} -> {args.out}")


if __name__ == "__main__":
    main()

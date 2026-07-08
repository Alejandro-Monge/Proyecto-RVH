"""matching_mensajes — MESSAGES-shadow ↔ GT matching (Section 6) + metrics Section 7.1/7.2/7.3.

Consumes the SHARED CORE of the matching (scripts/matching_core.py — the same
Section 6 specification, character by character, as the visual shadow) and the
SHARED crossing DETECTOR (the GT generator, offline, not included — the same
detector: front, the net's stop line, 3 m hysteresis, per transition). Single-
source doctrine: zero copies of logic.

SHADOW EVENTS: derived from the dense dump of the causal reconstructor
(one row per 0.1 s tick: x,y = commanded front; lane/edge mapped by SUMO
post-step). The player's crossing logger (crossing_times.csv) is NOT used:
a different detector (J5 polygon, no hysteresis) — it would measure a difference
of detectors, not of shadow vs reality.

ENTRY ARM of the shadow: first NORMAL approach edge
if it exists; if the vehicle never touches approach (it is born inside the
crossing, e.g. v119), it is derived from the initial INTERNAL lane via
lane_map.junction_internal. The detector is applied against the line of the arm
consistent with the movement.

CENSORING (protocol Section 2.3, per transition): a vehicle whose first
representable position is already past the line does NOT generate a crossing
event.
  - v0/v1/v2: censored ALSO in the GT (born past on both sides) →
    symmetric discount, outside the matching universe, they are NOT omissions.
  - Shadow-censored with an event in the GT (v119 at 2 Hz): **OMISSION**
    (Section 6.4/7.3) — a real asymmetry of the channel (crossing within the
    first interval between messages; a cadence-dependent failure mode, hook
    Section 8.1). Traceable per vehicle in omisiones_mensajes.csv
    (cause = cruce_en_primer_intervalo).

WINDOW: config/sites/rusia/matching_mensajes.yaml (1.5 s, justified by the
Section 6.3 sweep — plateau [1.25, 3.0]; the sweep is re-emitted on each run).

IDENTITY CONTROL (verification only, NEVER a mechanism): shadow and GT
share a physical id (veh_N ↔ vN); id_ok is reported per matched pair.

Usage:
  python3 scripts/matching_mensajes.py \
      --dense runs/<corrida>/results/denso.csv.gz --out runs/<corrida>/matching
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parent.parent
sys.path.insert(0, str(RAIZ / "scripts"))
import sumolib

from cruce_detector import cruce, linea
from matching_core import audit_ties, per_arm_report, run_matching

GT_CSV = RAIZ / "data/ground_truth/rusia/gt_mensajes_sim_rusia_carril.csv"
LANE_MAP = RAIZ / "config/sites/rusia/sumo/lane_map.yaml"
CFG = RAIZ / "config/sites/rusia/matching_mensajes.yaml"
NET_XML = RAIZ / "reconstruccion_causal/inputs/intersection_rusia.net.xml"

ARMS = ["este", "norte", "oeste", "sur"]
ENTRY_EDGE = {"sur": "E5", "norte": "E6", "este": "E3", "oeste": "E1"}
EDGE_ARM = {v: k for k, v in ENTRY_EDGE.items()}
EXIT_EDGE = {a: f"-{e}" for a, e in ENTRY_EDGE.items()}
EXIT_ARM = {v: k for k, v in EXIT_EDGE.items()}


def cargar_junction_internal():
    """internal edge (':J5_9') -> (entry_arm, exit_arm), from lane_map.yaml
    (junction_internal maps movement -> internal lane; indexed by edge)."""
    with open(LANE_MAP) as f:
        jm = yaml.safe_load(f)["junction_internal"]
    out = {}
    for mov, lane_id in jm.items():
        e_arm, x_arm = mov.split("->")
        out[lane_id.rsplit("_", 1)[0]] = (e_arm, x_arm)
    return out


def eventos_sombra(dense: pd.DataFrame, net):
    """Derives the shadow's crossing events from the dense dump, with the shared
    detector. Returns (eventos_df, censurados_sombra_df)."""
    internos = cargar_junction_internal()
    eventos, censurados = [], []
    for veh, g in dense.groupby("vehicle_id"):
        g = g.sort_values("sim_time_s")
        morro = g[["x", "y"]].to_numpy(float)
        t = g.sim_time_s.to_numpy(float)
        lanes = g.lane_sumo.astype(str).tolist()
        edges = [ln.rsplit("_", 1)[0] if ln and ln != "nan" else "" for ln in lanes]
        # entry arm: first normal approach; if none, movement of the
        # first internal edge (v119 is born inside the crossing)
        entry_arm = next((EDGE_ARM[e] for e in edges if e in EDGE_ARM), None)
        via = "approach"
        if entry_arm is None:
            entry_arm = next((internos[e][0] for e in edges if e in internos), None)
            via = "interno"
        # exit arm: last normal departure; if none, movement of the last internal one
        exit_arm = next((EXIT_ARM[e] for e in reversed(edges) if e in EXIT_ARM), None)
        if exit_arm is None:
            exit_arm = next((internos[e][1] for e in reversed(edges) if e in internos), None)
        if entry_arm is None:
            censurados.append(dict(vehicle_id=veh, causa="sin_brazo_resoluble",
                                   primer_lane=lanes[0], t0=round(float(t[0]), 2)))
            continue
        la, lb = linea(net, ENTRY_EDGE[entry_arm], "fin")
        t_pare, cens = cruce(morro, t, la, lb, morro[-1])
        if t_pare is None:
            censurados.append(dict(
                vehicle_id=veh, causa="nace_pasado_la_linea" if cens else "no_cruza",
                entry_arm=entry_arm, via_brazo=via,
                primer_lane=lanes[0], t0=round(float(t[0]), 2)))
            continue
        # junction exit + observed lanes (SUMO lane from the dense dump)
        t_sal = None
        if exit_arm is not None:
            lxa, lxb = linea(net, EXIT_EDGE[exit_arm], "inicio")
            t_sal, _ = cruce(morro, t, lxa, lxb, morro[-1])
        entry_lane = next((int(ln.rsplit("_", 1)[1]) + 1 for ln, e in zip(lanes, edges)
                           if e in EDGE_ARM), None)
        exit_lane = next((int(ln.rsplit("_", 1)[1]) + 1
                          for ln, e in zip(reversed(lanes), reversed(edges))
                          if e in EXIT_ARM), None)
        eventos.append(dict(
            vehicle_id=veh, entry_arm=entry_arm, via_brazo=via,
            entry_time_s=round(t_pare, 4),
            exit_arm=exit_arm or "", exit_time_s=round(t_sal, 4) if t_sal is not None
            else round(float(t[-1]), 4), exit_censurado=t_sal is None,
            entry_lane=entry_lane, exit_lane=exit_lane))
    return pd.DataFrame(eventos), pd.DataFrame(censurados)


def metricas_71(pares, gt_m, n_gt_arm):
    """Section 7.1 three levels + P/R/F1 per arm (same definition as the visual;
    EXACT lane: in the sim there are no bands — per_lane in the 4 arms)."""
    gt_idx = gt_m.set_index("gt_vehicle_id")
    rows = []
    for arm in ARMS + ["TOTAL"]:
        sub = pares if arm == "TOTAL" else pares[pares.entry_arm == arm]
        g = sum(n_gt_arm.values()) if arm == "TOTAL" else n_gt_arm.get(arm, 0)
        m = len(sub)
        e = f = 0
        for r in sub.itertuples():
            grow = gt_idx.loc[r.gt_vehicle_id]
            el = (r.entry_lane_sh is not None and not pd.isna(r.entry_lane_sh)
                  and int(r.entry_lane_sh) == int(grow.entry_lane))
            xa = r.sh_exit == grow.exit_arm
            xl = (r.exit_lane_sh is not None and not pd.isna(r.exit_lane_sh)
                  and str(grow.exit_lane) not in ("", "nan")
                  and int(r.exit_lane_sh) == int(float(grow.exit_lane)))
            e += el
            f += el and xa and xl
        rows.append(dict(brazo=arm, gt=g, casados=m,
                         nivel1_brazo=round(m / g, 3) if g else float("nan"),
                         nivel2_mas_carril=round(e / g, 3) if g else float("nan"),
                         nivel3_ruta_completa=round(f / g, 3) if g else float("nan"),
                         carril_ok=e, ruta_ok=f))
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dense", required=True, help="denso.csv[.gz] del causal_player")
    ap.add_argument("--out", required=True)
    ap.add_argument("--window", type=float, default=None,
                    help="override de la ventana (default: matching_mensajes.yaml)")
    ap.add_argument("--freq", choices=["1Hz", "2Hz", "5Hz"], default=None,
                    help="usa la ventana POR FRECUENCIA de config (windows:), "
                         "calibrada P00∩P50+0.2 — modo barrido MC")
    ap.add_argument("--no-sweep", action="store_true",
                    help="omite el barrido §6.3 (producción masiva; el sweep es "
                         "de calibración — la alerta de proximidad al borde del "
                         "agregador vigila la ventana por condición)")
    args = ap.parse_args()

    with open(CFG) as f:
        cfg_y = yaml.safe_load(f)
    if args.window is not None:
        window = args.window
    elif args.freq is not None:
        window = float(cfg_y["windows"][args.freq])
    else:
        window = float(cfg_y["matching_tolerance_s"])
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    net = sumolib.net.readNet(str(NET_XML))
    dense = pd.read_csv(args.dense)
    sh, cens_sh = eventos_sombra(dense, net)

    gt = pd.read_csv(GT_CSV)
    gt_cens = gt[gt.cruce_censurado == "si"]
    gt_m = gt[gt.cruce_censurado == "no"].rename(columns={
        "cruce_pare_sim_s": "entry_time_s", "salida_junction_sim_s": "exit_time_s",
        "vehicle_id": "gt_vehicle_id"}).copy()
    gt_m["entry_time_s"] = gt_m.entry_time_s.astype(float)
    gt_m["exit_time_s"] = gt_m.exit_time_s.astype(float)

    # symmetric censoring (GT and shadow censored = same physical vehicles):
    # outside the universe, NOT omissions. Shadow-censored WITH a GT event: stays
    # IN the GT universe → falls as a traceable OMISSION (v119: cadence effect).
    ids_gt_cens = set(gt_cens.vehicle_id)
    cens_sh["vehicle_id_gt"] = cens_sh.vehicle_id.str.replace("veh_", "v", regex=False)
    simetricos = set(cens_sh.vehicle_id_gt) & ids_gt_cens

    pares, fant, omis, contested = run_matching(
        sh, gt_m, ARMS, window, sh_id_col="vehicle_id", gt_id_col="gt_vehicle_id")

    # identity control (verification, not a mechanism)
    pares["id_ok"] = pares.vehicle_id.str.replace("veh_", "v", regex=False) == pares.gt_vehicle_id
    # shadow lanes for the pair (for Section 7.1)
    sh_idx = sh.set_index("vehicle_id")
    pares["entry_lane_sh"] = [sh_idx.loc[v].entry_lane for v in pares.vehicle_id]
    pares["exit_lane_sh"] = [sh_idx.loc[v].exit_lane for v in pares.vehicle_id]

    # traceable omissions: cause per vehicle
    causas = []
    for r in omis.itertuples():
        vid_sh = r.gt_vehicle_id.replace("v", "veh_", 1)
        if (cens_sh.vehicle_id == vid_sh).any():
            causas.append("cruce_en_primer_intervalo")   # shadow censored, GT not (v119)
        elif vid_sh in set(dense.vehicle_id):
            causas.append("no_casado_en_ventana")
        else:
            causas.append("sin_sombra")
    omis["causa"] = causas

    n_gt_arm = gt_m.groupby("entry_arm").size().to_dict()
    rep = per_arm_report(pares, fant, omis, sh, gt_m, ARMS)
    m71 = metricas_71(pares, gt_m, n_gt_arm)
    # P/R/F1 Section 7.3 (from the Section 6.4 report)
    rep["f1"] = round(2 * rep.precision * rep.recall / (rep.precision + rep.recall), 3)

    # Section 7.2 delay = cruce_pare_sombra − cruce_pare_GT (dt_entry, signed)
    d = pares.dt_entry
    delay = dict(n=len(d), mediana_s=round(d.median(), 3), media_s=round(d.mean(), 3),
                 std_s=round(d.std(), 3), p10_s=round(d.quantile(0.10), 3),
                 p90_s=round(d.quantile(0.90), 3), min_s=round(d.min(), 3),
                 max_s=round(d.max(), 3))
    delay_arm = pares.groupby("entry_arm").dt_entry.describe(
        percentiles=[.1, .5, .9])[["count", "10%", "50%", "90%"]].round(3)

    # Section 6.3 sweep (calibration; in the massive MC sweep it goes with --no-sweep)
    sweep = []
    for w in ([] if args.no_sweep else np.round(np.arange(0.5, 3.0001, 0.1), 2)):
        p, f, o, c = run_matching(sh, gt_m, ARMS, float(w),
                                  sh_id_col="vehicle_id", gt_id_col="gt_vehicle_id")
        idok = int((p.vehicle_id.str.replace("veh_", "v", regex=False)
                    == p.gt_vehicle_id).sum()) if len(p) else 0
        sweep.append(dict(ventana_s=w, casados=len(p), fantasmas=len(f), omisiones=len(o),
                          disputados=c,
                          dt_entry_mediana=round(p.dt_entry.median(), 3) if len(p) else float("nan"),
                          id_ok=idok))
    sweep_df = pd.DataFrame(sweep)
    if len(sweep_df):
        sweep_df.to_csv(out / "window_sweep_mensajes.csv", index=False)

    # --- artifacts ---
    pares.to_csv(out / "matches_mensajes.csv", index=False)
    fant.to_csv(out / "fantasmas_mensajes.csv", index=False)
    omis.to_csv(out / "omisiones_mensajes.csv", index=False)
    cens_sh.to_csv(out / "censura_sombra.csv", index=False)
    rep.to_csv(out / "baseline_por_brazo_mensajes.csv", index=False)
    m71.to_csv(out / "metricas_71_mensajes.csv", index=False)
    resumen = dict(
        universo=dict(gt_total=len(gt), gt_censurados=sorted(ids_gt_cens),
                      gt_emparejables=len(gt_m), sombra_vehiculos=int(dense.vehicle_id.nunique()),
                      sombra_eventos=len(sh),
                      censura_sombra=cens_sh.vehicle_id.tolist(),
                      censura_simetrica=sorted(simetricos)),
        ventana_s=window, casados=len(pares), fantasmas=len(fant), omisiones=len(omis),
        disputados=contested, empates_delta_banda=audit_ties(sh, gt_m, ARMS, window),
        control_identidad=dict(ok=int(pares.id_ok.sum()), total=len(pares),
                               discrepantes=pares[~pares.id_ok][
                                   ["vehicle_id", "gt_vehicle_id", "entry_arm",
                                    "dt_entry"]].to_dict("records")),
        delay_72=delay,
        insumos_md5={p.name: hashlib.md5(Path(p).read_bytes()).hexdigest()
                     for p in [Path(args.dense), GT_CSV, CFG, LANE_MAP]},
    )
    with open(out / "resumen_matching.json", "w") as f:
        json.dump(resumen, f, indent=2, ensure_ascii=False)

    print(f"[matching_mensajes] ventana={window}s  casados={len(pares)}  "
          f"fantasmas={len(fant)}  omisiones={len(omis)}  "
          f"id_ok={int(pares.id_ok.sum())}/{len(pares)}")
    print(rep.to_string(index=False))
    print("\n§7.1 (carril EXACTO, per_lane):")
    print(m71.to_string(index=False))
    print(f"\n§7.2 delay (sombra−GT, s): {delay}")
    print("por brazo:\n" + delay_arm.to_string())
    print(f"\nomisiones por causa: {omis.causa.value_counts().to_dict() if len(omis) else {}}")
    print(f"-> {out}")


if __name__ == "__main__":
    main()

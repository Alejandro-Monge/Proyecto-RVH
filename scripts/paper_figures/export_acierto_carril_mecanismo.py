"""SHADOW lane accuracy (B, drawn in SUMO) broken down BY MECHANISM — read-only.

Motivation: B can be correct without the shadow classifying anything (net topology / canonical
fallback coinciding with the tramo GT). A "lane hit" that mixes real classification
with match-by-fallback is misleading for the paper. This script separates the mechanisms.

On the COMPLETE-MATCH cases of the CAUSAL pipeline: B = getLaneID after step (golden rule, from
the causal_sumo_lanes.csv dump); A and GT from the current instrument; the MECHANISM is derived
by replicating the real production decision (`run_core` → group_carriles/exit_carril →
`choose_connection` over the net connections, queried from live SUMO), without touching it.

MECHANISMS (key column):
  clasificacion-observada-discriminante — the drawn lane was DETERMINED by the observed datum
      (entry classified per-lane or disambiguated by egress; exit chosen among several
      possible connections by the observed exit lane).
  topologia-unica-conexion — the net offered only one option from the chosen entry (the
      classifier was redundant or was ignored); includes the south default entry when
      the connection exists.
  fallback-canonico — observed tuple impossible in the net (orphan) or default entry with no
      connection → canonical connection of the movement. No classification merit.
  agrupado-sin-desambiguar — entry of the west {2,3} group with no turn to disambiguate: the
      drawn lane comes from the group, not from an individual classification.

Outputs: datos/acierto_carril_sombra.csv (per vehicle, entry AND exit) and
datos/acierto_carril_sombra_resumen.csv (per approach/frame: hits by classification,
hits by fallback/topology, failures).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common  # noqa: E402

INPUTS = [
    "verify_out/tracks_rusia.csv",
    "verify_out/events/rusia/track_events.csv",
    "verify_out/events/rusia/reconnected/matches.csv",
    "verify_out/events/rusia/shadow_gt_matches.csv",
    "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv",
    "config/sites/rusia/sumo/intersection_rusia.net.xml",
    "config/sites/rusia/sumo/lane_map.yaml",
    "digital_shadow/sumo/reconstruct.py",
    "scripts/run_harness_viz.py",
]
LANES_B = "verify_out/imagenes_finales/datos/causal_sumo_lanes.csv"


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    stamp[LANES_B] = common.md5_of(LANES_B)

    from run_harness_viz import run_core  # production, read-only
    from digital_shadow.sumo.lane_map import load_lane_map
    from digital_shadow.sumo.reconstruct import _movement_connections
    from digital_shadow.sumo.stage import SumoStage

    veh, _H = run_core()
    m = pd.read_csv(ROOT / "verify_out/events/rusia/shadow_gt_matches.csv")
    comp = m[m.categoria == "complete"].copy()
    b = pd.read_csv(ROOT / LANES_B).set_index("vid")

    lm = load_lane_map("rusia")
    stage = SumoStage(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml",
                      gui=False).start()
    tr = stage._traci
    conns_cache: dict[tuple, list] = {}

    def conns_of(ea, xa):
        if (ea, xa) not in conns_cache:
            conns_cache[(ea, xa)] = _movement_connections(tr, lm, ea, xa)
        return conns_cache[(ea, xa)]

    rows = []
    for r in comp.itertuples():
        v = veh.get(r.sys_vid)
        if v is None or r.sys_vid not in b.index:
            continue
        group = v["group_carriles"]                      # after disambiguation by egress
        label = v["lane_label"]                          # raw from the classifier ('', '2-3', '2', …)
        xc = v.get("exit_carril")                        # classified OBSERVED exit (or None)
        conns = conns_of(v["entry_arm"], v["exit_arm"])
        cand = [c for c in conns if c["from_carril"] in group]
        pref = [c for c in cand if c["to_carril"] == xc]
        arm_only = (label == "")
        agrupado = len(group) > 1
        desambiguado = ("-" in label) and not agrupado   # group reduced to 1 by observed turn

        # --- ENTRY mechanism (where the drawn from_carril comes from) ---
        if not cand:
            mec_e, det_e = "fallback-canonico", "tupla huérfana → canónica del movimiento"
        elif arm_only:
            mec_e, det_e = "topologia-unica-conexion", "entrada default [1] (arm_only), conexión existe"
        elif agrupado:
            mec_e, det_e = "agrupado-sin-desambiguar", f"grupo {label} sin giro que desambigüe"
        else:
            mec_e = "clasificacion-observada-discriminante"
            det_e = "desambiguado-por-egreso" if desambiguado else "carril de entrada clasificado"

        # --- EXIT mechanism (where the drawn to_carril comes from) ---
        to_opts = sorted({c["to_carril"] for c in cand}) if cand else []
        if not cand:
            mec_x, det_x = "fallback-canonico", "canónica del movimiento"
        elif pref and len(to_opts) > 1:
            mec_x, det_x = "clasificacion-observada-discriminante", \
                f"salida observada c{xc} elegida entre opciones {to_opts}"
        elif len(to_opts) == 1:
            mec_x = "topologia-unica-conexion"
            det_x = ("única opción; clasificador redundante" if pref
                     else f"única opción; ignora clasificación c{xc}")
        else:
            mec_x, det_x = "topologia-unica-conexion", \
                f"clasificación c{xc} sin conexión; cand[0] de {to_opts}"

        gt_e, gt_x = int(r.gt_entry_lane), int(r.gt_exit_lane)
        be = b.loc[r.sys_vid, "entry_carril_drawn"]
        bx = b.loc[r.sys_vid, "exit_carril_drawn"]
        exit_1lane = len(lm.arms[v["exit_arm"]].exit_lanes) == 1
        for marco, mec, det, bb, gg, a_lbl in (
                ("entrada", mec_e, det_e, be, gt_e, label or "arm_only"),
                ("salida", mec_x, det_x + (" · egreso de 1 carril (trivial)" if exit_1lane else ""),
                 bx, gt_x, "" if xc is None else str(xc))):
            ok = (int(bb) == gg) if pd.notna(bb) else None
            rows.append(dict(
                vid=r.sys_vid, movimiento=f"{v['entry_arm']}→{v['exit_arm']}",
                marco=marco, brazo=v["entry_arm"] if marco == "entrada" else v["exit_arm"],
                brazo_entrada=v["entry_arm"],
                carril_A=a_lbl, carril_B=(int(bb) if pd.notna(bb) else ""), carril_GT=gg,
                mecanismo=mec, detalle=det,
                b_eq_gt=("sí" if ok else "no") if ok is not None else "sin-lectura",
                acierto_por=("clasificación" if mec == "clasificacion-observada-discriminante"
                             else "fallback/topología") if ok else ""))
    stage.close()

    df = pd.DataFrame(rows)
    common.ensure_dirs()
    out = common.DATA_DIR / "acierto_carril_sombra.csv"
    df.to_csv(out, index=False)

    # --- summary per approach/frame: (a) classification, (b) fallback/topology, (c) failures ---
    # instrument convention: EVERYTHING grouped by ENTRY approach (like carril_metrics)
    res = []
    for marco in ("entrada", "salida"):
        for arm in ("este", "norte", "oeste", "sur"):
            sub = df[(df.marco == marco) & (df.brazo_entrada == arm) & (df.b_eq_gt != "sin-lectura")]
            ok = sub[sub.b_eq_gt == "sí"]
            res.append(dict(
                marco=marco, brazo_entrada=arm, n=len(sub),
                aciertos_B=len(ok),
                por_clasificacion=int((ok.acierto_por == "clasificación").sum()),
                por_fallback_topologia=int((ok.acierto_por == "fallback/topología").sum()),
                fallos=int((sub.b_eq_gt == "no").sum())))
    resumen = pd.DataFrame(res)
    out2 = common.DATA_DIR / "acierto_carril_sombra_resumen.csv"
    resumen.to_csv(out2, index=False)

    stamp_path = common.DATA_DIR / "provenance_acierto_carril_sombra.txt"
    stamp_path.write_text("\n".join([
        "volcado: acierto_carril_sombra.csv + _resumen.csv",
        "pipeline: CAUSAL; B = getLaneID tras step (causal_sumo_lanes); mecanismo replicando "
        "choose_connection de producción sobre SUMO vivo",
        *(f"  {p} = {mm}" for p, mm in sorted(stamp.items())),
    ]) + "\n")

    print(df.mecanismo.value_counts().to_string(), "\n")
    print(resumen.to_string(index=False))
    print(f"\n[ok] {out.relative_to(ROOT)} ({len(df)} filas) · {out2.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

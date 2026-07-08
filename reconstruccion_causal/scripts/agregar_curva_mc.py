"""agregar_curva_mc — aggregates the matching of the 3003 MC runs into the Section 8.1 curve.

Per condition (frequency × loss level), over the ~100 reps: percentiles
p10/p50/p90 (a band, never a flat average) of the Section 7.1/7.2/7.3 metrics,
with the THREE SEPARATE CHANNELS (never summed with each other):

  1. CHANNEL (the Section 8.1 curve): clean universe = non-censored GT without
     line-waiters (59 − 3 = 56). Omissions by labeled cause:
     cruce_en_primer_intervalo (cadence, v119 family) / no_casado_en_ventana /
     sin_sombra.
  2. WAITING-LINE BOUNDARY ZONE (resolution (a)): every artifact that involves
     line_waiters (config; fixed mobility ⇒ same ids across the 3003) goes to
     its own columns.
  3. CENSORING EDGE: shadow-events of the GT-censored ones (v0/v1/v2, born IN
     BAND ±mm) — edge ghosts, not channel ones.

The shared physical ids are used ONLY to classify artifacts
(traceability); the matching never used them to match.

EDGE-PROXIMITY ALERT (compensates for --no-sweep):
per condition, the fraction of matches with |dt_entry| > window − margin
(0.2 s = the calibration margin). A high fraction = the window is short there; it
is REPORTED as an alert, the sweep is not re-run.

Outputs (runs/matching_mc/): curva_degradacion.csv (long, percentiles),
tabla_respaldo.csv (one row per condition), alertas_borde.csv.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parent.parent
BASE = RAIZ / "reconstruccion_causal/runs/matching_mc"
GT_CSV = RAIZ / "data/ground_truth/rusia/gt_mensajes_sim_rusia_carril.csv"
CFG = RAIZ / "config/sites/rusia/matching_mensajes.yaml"
MARGEN_BORDE = 0.2
ALERTA_FRAC = 0.05     # reporting threshold (median fraction of pairs stuck to the cutoff)

cfg = yaml.safe_load(open(CFG))
LW = set(cfg["line_waiters"])                       # boundary zone (GT ids)
gt = pd.read_csv(GT_CSV)
GT_CENS = set(gt[gt.cruce_censurado == "si"].vehicle_id)          # v0/v1/v2
gt_ok = gt[gt.cruce_censurado == "no"].set_index("vehicle_id")
UNIVERSO_LIMPIO = [v for v in gt_ok.index if v not in LW]         # 56
N_CLEAN = len(UNIVERSO_LIMPIO)
N_CLEAN_ARM = gt_ok.loc[UNIVERSO_LIMPIO].groupby("entry_arm").size().to_dict()


def fisico(veh_id: str) -> str:
    return veh_id.replace("veh_", "v", 1)


def metricas_rep(run: Path):
    """Metrics of ONE rep, with the three separate channels."""
    r = json.load(open(run / "resumen_matching.json"))
    m = pd.read_csv(run / "matches_mensajes.csv")
    om = pd.read_csv(run / "omisiones_mensajes.csv") if (run / "omisiones_mensajes.csv").stat().st_size > 2 else pd.DataFrame()
    fa = pd.read_csv(run / "fantasmas_mensajes.csv") if (run / "fantasmas_mensajes.csv").stat().st_size > 2 else pd.DataFrame()
    w = float(r["ventana_s"])

    m["sh_fis"] = m.vehicle_id.map(fisico)
    en_lw = m.gt_vehicle_id.isin(LW) | m.sh_fis.isin(LW)
    mc = m[~en_lw]                                                # channel matches
    # Section 7.1 levels over the clean universe
    ok_lane = ok_ruta = 0
    for x in mc.itertuples():
        g = gt_ok.loc[x.gt_vehicle_id]
        el = pd.notna(x.entry_lane_sh) and int(x.entry_lane_sh) == int(g.entry_lane)
        xa = x.sh_exit == g.exit_arm
        xl = (pd.notna(x.exit_lane_sh) and str(g.exit_lane) not in ("", "nan")
              and int(x.exit_lane_sh) == int(float(g.exit_lane)))
        ok_lane += el
        ok_ruta += el and xa and xl
    # channel shadow (for precision): shadow events minus LW and minus edge-censoring
    n_sh = r["universo"]["sombra_eventos"]
    if len(fa):
        fa = fa.assign(sh_fis=fa.vehicle_id.map(fisico))
        fa_borde = int(fa.sh_fis.isin(GT_CENS).sum())
        fa_lw = int(fa.sh_fis.isin(LW).sum())
        fa_canal = len(fa) - fa_borde - fa_lw
    else:
        fa_borde = fa_lw = fa_canal = 0
    n_sh_canal = len(mc) + fa_canal
    if len(om):
        om_lw = int(om.gt_vehicle_id.isin(LW).sum())
        omc = om[~om.gt_vehicle_id.isin(LW)]
        om_cad = int((omc.causa == "cruce_en_primer_intervalo").sum())
        om_otras = len(omc) - om_cad
    else:
        om_lw = om_cad = om_otras = 0
    # direct v119 SIGNAL (independent of the matching): vehicles censored by the
    # SHADOW that the GT does observe = crossing within the first interval
    # between messages (cadence/loss). Robust to the Section 6 cost swaps, which
    # can re-label the omission (v119 surfaces as v117 via swap).
    cens_sombra = {fisico(v) for v in r["universo"]["censura_sombra"]}
    censura_cadencia = len(cens_sombra - GT_CENS - LW)

    recall = len(mc) / N_CLEAN
    precision = len(mc) / n_sh_canal if n_sh_canal else float("nan")
    borde_frac = float((mc.dt_entry.abs() > w - MARGEN_BORDE).mean()) if len(mc) else 0.0
    return dict(
        casados=len(mc), recall=recall, precision=precision,
        f1=2 * precision * recall / (precision + recall) if (precision + recall) else float("nan"),
        n2_carril=ok_lane / N_CLEAN, n3_ruta=ok_ruta / N_CLEAN,
        delay_mediana=float(mc.dt_entry.median()) if len(mc) else float("nan"),
        om_cadencia=om_cad, om_otras=om_otras, fant_canal=fa_canal,
        censura_cadencia=censura_cadencia,
        zl_matches=int(en_lw.sum()), zl_omisiones=om_lw, zl_fantasmas=fa_lw,
        borde_censura_fantasmas=fa_borde,
        frac_borde_ventana=borde_frac, ventana=w,
        id_ok=int(m[~en_lw].id_ok.sum()) if "id_ok" in m else None,
    )


def main():
    filas = []
    for run in sorted(BASE.glob("*/*/*/resumen_matching.json")):
        d = run.parent
        rep, lvl, freq = d.name, d.parent.name, d.parent.parent.name
        row = metricas_rep(d)
        row.update(freq=freq, nivel=lvl, rep=rep)
        filas.append(row)
    df = pd.DataFrame(filas)
    print(f"reps agregadas: {len(df)} en {df.groupby(['freq','nivel']).ngroups} condiciones")

    METRICAS = ["recall", "precision", "f1", "n2_carril", "n3_ruta", "delay_mediana",
                "om_cadencia", "om_otras", "fant_canal", "censura_cadencia",
                "zl_matches", "zl_omisiones", "zl_fantasmas", "borde_censura_fantasmas"]
    largo = []
    respaldo = []
    alertas = []
    for (freq, lvl), g in df.groupby(["freq", "nivel"]):
        base = dict(freq=freq, nivel=lvl, p_nominal=int(lvl[1:]), reps=len(g),
                    ventana_s=g.ventana.iloc[0])
        fila = dict(base)
        for met in METRICAS:
            v = g[met].astype(float)
            for p, tag in [(0.10, "p10"), (0.50, "p50"), (0.90, "p90")]:
                largo.append(dict(base, metrica=met, percentil=tag,
                                  valor=round(float(v.quantile(p)), 4)))
            fila[f"{met}_p10"] = round(float(v.quantile(0.10)), 4)
            fila[f"{met}_p50"] = round(float(v.quantile(0.50)), 4)
            fila[f"{met}_p90"] = round(float(v.quantile(0.90)), 4)
        fb = g.frac_borde_ventana.astype(float)
        fila["frac_borde_p50"] = round(float(fb.quantile(0.5)), 4)
        fila["frac_borde_max"] = round(float(fb.max()), 4)
        respaldo.append(fila)
        if fb.quantile(0.5) > ALERTA_FRAC:
            alertas.append(dict(base, frac_borde_p50=round(float(fb.quantile(0.5)), 4),
                                frac_borde_max=round(float(fb.max()), 4),
                                alerta="ventana posiblemente corta en esta condición"))
    pd.DataFrame(largo).sort_values(["freq", "p_nominal", "metrica", "percentil"]) \
        .to_csv(BASE / "curva_degradacion.csv", index=False)
    pd.DataFrame(respaldo).sort_values(["freq", "p_nominal"]) \
        .to_csv(BASE / "tabla_respaldo.csv", index=False)
    al = pd.DataFrame(alertas)
    al.to_csv(BASE / "alertas_borde.csv", index=False)
    print(f"-> curva_degradacion.csv ({len(largo)} filas), tabla_respaldo.csv "
          f"({len(respaldo)} condiciones), alertas_borde.csv ({len(al)} alertas)")
    if len(al):
        print(al.to_string(index=False))
    else:
        print("alerta de proximidad al borde: NINGUNA condición supera el umbral "
              f"({ALERTA_FRAC:.0%} mediano de pares a <{MARGEN_BORDE}s del corte)")


if __name__ == "__main__":
    main()

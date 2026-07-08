"""comparar_ocupacion_mensajes — consumer of the Section 7.7 comparator for the
MESSAGES shadow (agnostic core: scripts/occupancy_compare.py).

SCOPE: it closes the occupancy comparison ONLY for messages. The visual one is
still pending its series adapter (declared, not built).

STRUCTURAL CAVEAT ON `detenidos` — IN THE OUTPUT, not only in the docs: under
DR-until-anchor, a real stopped vehicle that loses
signal keeps being extrapolated with the past velocity ⇒ shadow-stopped is NOT
equivalent to real stops and its bias GROWS with the loss (occupancy_mensajes.md
Section 6.1). Each stopped row of the summary carries the note; the JSON carries
the `salvedad_detenidos` field. Occupancy and stopped are NOT equivalent metrics.

Usage:
  python3 scripts/comparar_ocupacion_mensajes.py \
      --sombra runs/<id>/results/ocupacion.csv --out runs/<id>/comparacion
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import pandas as pd

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parent.parent
sys.path.insert(0, str(RAIZ / "scripts"))
from occupancy_compare import compare_series  # noqa: E402

GT_DEFAULT = RAIZ / "reconstruccion_causal/runs/gt2_ocupacion/gt2_ocupacion.csv"

SALVEDAD = ("detenidos-sombra tiene SESGO ESTRUCTURAL bajo DR: no equivale a "
            "paradas reales y crece con la perdida (occupancy_mensajes.md §6.1). "
            "NO interpretar como equivalente a ocupacion.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sombra", required=True, help="ocupacion.csv del causal_player")
    ap.add_argument("--gt", default=str(GT_DEFAULT))
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    sombra = pd.read_csv(args.sombra)
    gt = pd.read_csv(args.gt)
    res = compare_series(sombra, gt)

    # caveat IN the output: note column in the summary (stopped rows)
    summ = res["summary"].copy()
    summ["nota"] = summ.metrica.map(lambda m: SALVEDAD if m == "detenidos" else "")
    summ.to_csv(out / "resumen_comparacion.csv", index=False)
    res["aligned"].to_csv(out / "serie_alineada.csv", index=False)
    res["mae_lag"].to_csv(out / "mae_lag.csv", index=False)
    meta = dict(
        grid_report=res["grid_report"],
        salvedad_detenidos=SALVEDAD,
        insumos_md5={Path(p).name: hashlib.md5(Path(p).read_bytes()).hexdigest()
                     for p in (args.sombra, args.gt)},
    )
    with open(out / "grid_report.json", "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print(f"[comparador] rejilla OK: {res['grid_report']['ticks_comunes']} ticks comunes "
          f"{res['grid_report']['rango_comun']}; recorte declarado: "
          f"solo_gt={res['grid_report']['recorte_solo_gt']['n']}, "
          f"solo_sombra={res['grid_report']['recorte_solo_sombra']['n']}")
    br = summ[summ.nivel == "brazo"]
    print(br[["metrica", "brazo", "mae_0", "lag_star_s", "mae_lag_star", "delta",
              "exact_0", "exact_lag_star"]].to_string(index=False))
    print(f"\nSALVEDAD detenidos: {SALVEDAD}")
    print(f"-> {out}")


if __name__ == "__main__":
    main()

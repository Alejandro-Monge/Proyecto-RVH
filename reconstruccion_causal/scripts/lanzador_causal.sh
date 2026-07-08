#!/usr/bin/env bash
# lanzador_causal.sh — menu to launch runs of the CAUSAL reconstructor
# over the own Monte Carlo sweep (mobility f712b746).
#
# Adaptation of the launcher of the previous base version (not included in this repository):
# its proven flow is preserved (menu type -> frequency ->
# level -> mode, derivation of period by frequency, CSV existence
# check, out with timestamp, DRY_RUN) and the specific parts are changed:
#   - data: montecarlo/<f>Hz/P<level>/rep###.csv (before: the collaborator's
#     Barridos_veins_perdida, old mobility) with an automatic ID shim;
#   - player: causal_player.py (guard assert; the per-slot engine IS the
#     reconstruction: there is no --reconstruct flag, and --period-s is always passed);
#   - new modes: besides the headless and the simple GUI, the two
#     reality-vs-shadow comparison views (two windows / overlaid);
#   - levels: P00..P50 in steps of 5 (the MC sweep), with a selectable rep.
#
# Usage: ./scripts/lanzador_causal.sh
#        DRY_RUN=1 ./scripts/lanzador_causal.sh   (shows the command, does not run)
set -euo pipefail
cd "$(dirname "$0")/.."

MC=../via_b_runtime/RusiaV2IViaB/results/montecarlo
NET=inputs/intersection_rusia.net.xml

ts() { date +%Y%m%d-%H%M; }
in_list() { local x="$1"; shift; for v in "$@"; do [ "$x" = "$v" ] && return 0; done; return 1; }

echo "=============================================="
echo "  Lanzador — Reconstructor CAUSAL (guardia 1s)"
echo "=============================================="

# 1) mode
echo
echo "Modo:"
echo "  1) Certificación headless (assert, summary + validation_log)"
echo "  2) Vista DOS VENTANAS realidad|sombra (preferida)"
echo "  3) Vista SUPERPUESTA (un SUMO, sincronía por barrera)"
echo "  4) GUI simple de la sombra sola"
echo "  5) Vista DOS VENTANAS + SOMBRA 3D WEB (tiempo real 1x; imprime URL)"
read -rp "Elige [1/2/3/4/5]: " modo
in_list "$modo" 1 2 3 4 5 || { echo "Opción inválida."; exit 1; }

# 2) frequency -> period
echo
read -rp "Frecuencia [1 / 2 / 5] Hz: " fq
case "$fq" in
  1) period=1.0 ;;
  2) period=0.5 ;;
  5) period=0.2 ;;
  *) echo "Frecuencia inválida."; exit 1 ;;
esac

# 3) level + rep -> source csv
echo
read -rp "Nivel de pérdida [00/05/10/15/20/25/30/35/40/45/50]: " lv
in_list "$lv" 00 05 10 15 20 25 30 35 40 45 50 || { echo "Nivel inválido."; exit 1; }
if [ "$lv" = "00" ]; then
  src="$MC/${fq}Hz/P00/base.csv"     # P00 = no loss: a single base file
  rep="base"
else
  read -rp "Rep [000..] (default 000): " rep
  rep=${rep:-000}
  src="$MC/${fq}Hz/P${lv}/rep${rep}.csv"
fi
[ -f "$src" ] || { echo "No existe el CSV: $src"; exit 1; }

# ID shim (v7 -> veh_7), only once per combo
dst="inputs/cert/mc_${fq}Hz_P${lv}_${rep}.csv"
if [ ! -f "$dst" ]; then
  echo ">> preparando insumo (shim de IDs): $dst"
  [ -n "${DRY_RUN:-}" ] || python3 scripts/preparar_entrada.py --src "$src" --dst "$dst"
fi

out="runs/lanzador/${fq}Hz/P${lv}/$(ts)_rep${rep}"
avisar_gui() {
  echo "NOTA: los modos GUI usan staging 'park' + clamp al bbox (el assert NaN"
  echo "      de sumo-gui 1.12 con posiciones fuera de red quedó mitigado)."
}

case "$modo" in
  1) cmd="python3 scripts/causal_player.py --csv $dst --net $NET --out $out \
--delay-s 1.0 --period-s $period --interp linear --guard-mode assert" ;;
  2) avisar_gui
     cmd="CSV=$dst PERIOD=$period OUT=$out ./scripts/vista_dos_ventanas.sh" ;;
  3) avisar_gui
     cmd="CSV=$dst PERIOD=$period OUT=$out ./scripts/vista_comparada.sh" ;;
  4) avisar_gui
     cmd="python3 scripts/causal_player.py --csv $dst --net $NET --out $out \
--delay-s 1.0 --period-s $period --interp linear --guard-mode assert --staging-mode park --gui" ;;
  5) avisar_gui
     cmd="CSV=$dst PERIOD=$period OUT=$out ./scripts/vista_dos_ventanas_3d.sh" ;;
esac

echo
echo ">> $cmd"
echo
if [ -n "${DRY_RUN:-}" ]; then echo "(DRY_RUN: no se ejecuta)"; exit 0; fi
eval "$cmd"

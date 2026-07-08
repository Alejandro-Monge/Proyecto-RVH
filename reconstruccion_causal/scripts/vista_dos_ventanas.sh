#!/usr/bin/env bash
# vista_dos_ventanas.sh — REALITY and SHADOW in TWO SUMO-GUI side by side.
#
#   LEFT window (REALITY, port 8892): driver_replay.py (INTACT) with
#       the GT2 mobility f712b746, default SUMO colors, IDs v7.
#   RIGHT window  (SHADOW,   port 8893): causal_player.py --attach,
#       strict 1 s buffer with the guard in assert, red shadows, IDs veh_7.
#
# Both SUMO run with step 0.05 s and the SAME --delay (ms of render per
# step), so the two clocks advance at the same wall pace. INHERENT
# LIMITATION of the two-window setup: the synchronization is by pace, not by
# barrier — pausing or changing the delay of a SINGLE window desyncs them.
# For hard synchronization in a single overlaid view: scripts/vista_comparada.sh.
#
# Usage:
#   ./scripts/vista_dos_ventanas.sh                    # GUI, P30, delay 1000 ms
#   DELAY=200 CSV=inputs/mc_2Hz_P00_base.csv ./scripts/vista_dos_ventanas.sh
#   MODE=headless SMOKE_TMAX=30 ./scripts/vista_dos_ventanas.sh   # smoke
set -euo pipefail
cd "$(dirname "$0")/.."

PORT_REAL=${PORT_REAL:-8892}
PORT_SOMBRA=${PORT_SOMBRA:-8893}
CSV=${CSV:-inputs/mc_2Hz_P30_rep000.csv}
NET=inputs/intersection_rusia.net.xml
PERIOD=${PERIOD:-0.5}
OUT=${OUT:-runs/vista2_$(date +%Y%m%d-%H%M)}
DELAY=${DELAY:-1000}              # ms of render per 0.05 s step (1000 = 20x slow)
MODE=${MODE:-gui}                 # gui | headless (smoke)
SMOKE_TMAX=${SMOKE_TMAX:-}
DRIVER_DIR=../reconstruccion_veins

BIN=sumo-gui
GUI_REAL="--delay $DELAY --start --window-size 950,950 --window-pos 0,0"
GUI_SOMBRA="--delay $DELAY --start --window-size 950,950 --window-pos 960,0"
[ "$MODE" = headless ] && { BIN=sumo; GUI_REAL=""; GUI_SOMBRA=""; }

COMUN="--step-length 0.05 --time-to-teleport -1 --collision.action none \
       --ignore-route-errors --no-step-log true --end 100000"

echo "[vista2] REALIDAD :$PORT_REAL | SOMBRA :$PORT_SOMBRA | csv=$CSV | delay=${DELAY}ms | out=$OUT"
$BIN -n "$NET" $COMUN --remote-port "$PORT_REAL"   $GUI_REAL &
SUMO_REAL_PID=$!
$BIN -n "$NET" $COMUN --remote-port "$PORT_SOMBRA" $GUI_SOMBRA &
SUMO_SOMBRA_PID=$!
sleep 2

python3 scripts/causal_player.py --csv "$CSV" --net "$NET" --out "$OUT" \
    --delay-s 1.0 --period-s "$PERIOD" --interp linear --guard-mode assert \
    --traci-port "$PORT_SOMBRA" --client-order 1 --server-step 0.05 \
    --shadow-color 255,20,60,220 --staging-mode park &
CAUSAL_PID=$!

DRIVER_ARGS=""
[ -n "$SMOKE_TMAX" ] && DRIVER_ARGS="--t-max $SMOKE_TMAX"
( cd "$DRIVER_DIR" && TRACI_PORT=$PORT_REAL python3 driver_replay.py $DRIVER_ARGS ) &
DRIVER_PID=$!

if [ -n "$SMOKE_TMAX" ]; then
    wait "$DRIVER_PID" || true
    sleep 2
    kill "$SUMO_REAL_PID" "$SUMO_SOMBRA_PID" 2>/dev/null || true
    wait "$CAUSAL_PID" || true
    echo "[vista2] smoke terminado; salidas de la sombra en $OUT"
else
    wait "$DRIVER_PID" || true
    wait "$CAUSAL_PID" || true
    wait "$SUMO_REAL_PID" "$SUMO_SOMBRA_PID" || true
    echo "[vista2] sesión terminada; salidas de la sombra en $OUT"
fi

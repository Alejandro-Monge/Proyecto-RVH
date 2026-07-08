#!/usr/bin/env bash
# vista_comparada.sh — REALITY vs SHADOW in a single SUMO, same clock.
#
#   REALITY: driver_replay.py (reconstruccion_veins/, INTACT) — mobility
#             GT2 f712b746, 62 vehicles, TraCI client order=2, default SUMO
#             colors (yellow).
#   SHADOW:  causal_player.py in --attach mode, client order=1, strict 1 s
#             buffer with the guard in assert (if the session violates causality,
#             the shadow aborts and says so). All shadows in semi-transparent
#             RED to tell them apart by eye. IDs: v7 (reality)
#             vs veh_7 (shadow of the same vehicle).
#
# The SUMO runs with step 0.05 s and --num-clients 2: both clients are
# synchronized by construction (TraCI barrier), same clock. The shadow goes
# ~1 s behind reality BY DESIGN (delay=buffer); what is audited is
# where it additionally separates due to loss: freezes (HELD), removals by
# timeout, re-acquisitions.
#
# Usage:
#   ./scripts/vista_comparada.sh                     # GUI, P30
#   CSV=inputs/mc_2Hz_P00_base.csv ./scripts/vista_comparada.sh
#   MODE=headless SMOKE_TMAX=30 ./scripts/vista_comparada.sh   # smoke without GUI
set -euo pipefail
cd "$(dirname "$0")/.."

# 8873 is published by the rvh-veins container (MC sweep): the view uses another port
PORT=${PORT:-8891}
CSV=${CSV:-inputs/mc_2Hz_P30_rep000.csv}
NET=inputs/intersection_rusia.net.xml
PERIOD=${PERIOD:-0.5}
OUT=${OUT:-runs/vista_$(date +%Y%m%d-%H%M)}
MODE=${MODE:-gui}                 # gui | headless (smoke)
SMOKE_TMAX=${SMOKE_TMAX:-}        # if set: cuts the driver at t (smoke)
DRIVER_DIR=../reconstruccion_veins

DELAY=${DELAY:-1000}              # ms of render per step (1000 = 20x slow motion)
BIN=sumo-gui
GUIFLAGS="--delay $DELAY --start"
[ "$MODE" = headless ] && { BIN=sumo; GUIFLAGS=""; }

echo "[vista] SUMO $BIN :$PORT | csv=$CSV | out=$OUT"
$BIN -n "$NET" --step-length 0.05 --num-clients 2 --remote-port "$PORT" \
    --time-to-teleport -1 --collision.action none --ignore-route-errors \
    --no-step-log true --end 100000 $GUIFLAGS &
SUMO_PID=$!
sleep 2

python3 scripts/causal_player.py --csv "$CSV" --net "$NET" --out "$OUT" \
    --delay-s 1.0 --period-s "$PERIOD" --interp linear --guard-mode assert \
    --traci-port "$PORT" --client-order 1 --server-step 0.05 \
    --shadow-color 255,20,60,220 --staging-mode park &
CAUSAL_PID=$!

DRIVER_ARGS=""
[ -n "$SMOKE_TMAX" ] && DRIVER_ARGS="--t-max $SMOKE_TMAX"
( cd "$DRIVER_DIR" && TRACI_PORT=$PORT python3 driver_replay.py $DRIVER_ARGS ) &
DRIVER_PID=$!

if [ -n "$SMOKE_TMAX" ]; then
    # smoke: when the driver stops, SUMO is brought down and the shadow closes cleanly
    wait "$DRIVER_PID" || true
    sleep 2
    kill "$SUMO_PID" 2>/dev/null || true
    wait "$CAUSAL_PID" || true
    echo "[vista] smoke terminado; salidas de la sombra en $OUT"
else
    wait "$DRIVER_PID" || true
    wait "$CAUSAL_PID" || true
    wait "$SUMO_PID" || true
    echo "[vista] sesión terminada; salidas de la sombra en $OUT"
fi

#!/usr/bin/env bash
# grabar_sombra.sh — STEP 1 of the demo video: records the SHADOW panel
# (MESSAGES, V2X 5 Hz / 30% loss) with MANUAL FRAMING.
#
# Flow: opens a PAUSED sumo-gui with the shadow. YOU frame the view by hand
# (zoom/pan with the mouse) and press Enter in THIS terminal. From there the loop
# captures one shadow shot per sim (sh_<ms>.png) + writes the ws_log (for the 3D panel).
# The framing stays fixed for the whole capture (simulationStep does not move the view).
#
# It MUST run in the foreground (this terminal): the Enter comes in via stdin.
#
# Usage:  scripts/demo_causal/grabar_sombra.sh
#         OUT=/output/path  CSV=inputs/cert/mc_5Hz_P30_000.csv  scripts/demo_causal/grabar_sombra.sh
set -euo pipefail
cd "$(dirname "$0")/../.."            # -> implementacion/
REPO=$(pwd)
export SUMO_HOME=${SUMO_HOME:-/usr/share/sumo}
export DISPLAY=${DISPLAY:-:1}

OUT=${OUT:-$REPO/verify_out/demo_causal/mensajes}
CSV=${CSV:-inputs/cert/mc_5Hz_P30_000.csv}     # relative to reconstruccion_causal/
# view (color scheme) — from config, NOT the framing (which is manual)
SCHEME=$REPO/config/sites/rusia/$(python3 -c \
  "import yaml;print(yaml.safe_load(open('$REPO/config/sites/rusia/demo_view.yaml'))['scheme_file'])")

mkdir -p "$OUT/shots"
echo "[grabar_sombra] OUT=$OUT  CSV=$CSV  scheme=$SCHEME"
echo "[grabar_sombra] abriendo sumo-gui (sombra MENSAJES). Encuadra y pulsa Enter aquí."
cd "$REPO/reconstruccion_causal"
python3 scripts/causal_player.py \
  --csv "$CSV" --net inputs/intersection_rusia.net.xml --out "$OUT" \
  --delay-s 1.0 --period-s 0.2 --interp linear --guard-mode assert --staging-mode park \
  --gui --gui-settings-file "$SCHEME" \
  --shot-dir "$OUT/shots" --ws-log "$OUT/ws_log.jsonl" \
  --manual-frame
echo "[grabar_sombra] LISTO: shadow shots en $OUT/shots ; ws_log en $OUT/ws_log.jsonl"
echo "[grabar_sombra] siguiente: scripts/demo_causal/grabar_realidad.sh (una sola vez, se reusa)"

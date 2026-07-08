#!/usr/bin/env bash
# componer_mensajes.sh — STEP 3 of the demo video: with the two recordings
# already done (shadow + reality), it generates the 3D panel from the ws_log and COMPOSES the
# 3-panel MP4. Non-interactive (no framing here).
#
#   REALITY (GT2) | SHADOW (MESSAGES) | SHADOW 3D   — paired by sim, real[T]|shadow[T].
#
# Does: (1) brings up the 3D webserver; (2) replay_ws3d --all over the shadow ws_log
# (deterministic headless render, byte-identical to the live client, Part 1) -> one PNG per
# step; (3) compose_3panel -> MP4; (4) brings the webserver down.
#
# Usage:  SHADOW=verify_out/demo_causal/mensajes  REAL=verify_out/demo_causal/realidad_gt2 \
#         scripts/demo_causal/componer_mensajes.sh
set -euo pipefail
cd "$(dirname "$0")/../.."            # -> implementacion/
REPO=$(pwd)
export SUMO_HOME=${SUMO_HOME:-/usr/share/sumo}

SHADOW=${SHADOW:-$REPO/verify_out/demo_causal/mensajes}      # has shots/ + ws_log.jsonl
REAL=${REAL:-$REPO/verify_out/demo_causal/realidad_gt2}      # reality shots
OUT=${OUT:-$REPO/verify_out/demo_causal/demo_mensajes.mp4}
TITLE=${TITLE:-"MENSAJES (V2X 5 Hz, 30% perdida)"}
WSLOG="$SHADOW/ws_log.jsonl"
FRAMES3D=${FRAMES3D:-$SHADOW/frames3d}

for p in "$SHADOW/shots" "$REAL" "$WSLOG"; do
  [ -e "$p" ] || { echo "falta $p — corre antes grabar_sombra.sh y grabar_realidad.sh"; exit 1; }
done

# 3D HTTP port (config: web3d.yaml)
HTTP_PORT=$(python3 -c \
  "import yaml;print(yaml.safe_load(open('$REPO/config/sites/rusia/web3d.yaml'))['http_port'])")

echo "[componer] levantando webserver 3D (:$HTTP_PORT)…"
python3 -m ws_emitter.webserver --site rusia > /tmp/_web3d_mensajes.log 2>&1 &
WEB_PID=$!
trap 'kill $WEB_PID 2>/dev/null || true' EXIT
sleep 2

echo "[componer] render 3D headless (replay_ws3d --all)…"
rm -rf "$FRAMES3D"; mkdir -p "$FRAMES3D"
WIN=$(python3 -c \
  "import yaml;w=yaml.safe_load(open('$REPO/config/sites/rusia/demo_view.yaml'))['window_size'];print(w[0],w[1])")
read -r W H <<< "$WIN"
( cd "$REPO/scripts/demo_causal/recorder" && \
  node replay_ws3d.mjs --ws-log "$WSLOG" \
    --url "http://127.0.0.1:$HTTP_PORT/dev_replay.html" \
    --out "$FRAMES3D" --width "$W" --height "$H" --all )

# crop of the SUMO panels (config: demo_view.yaml, tied to the recording's framing)
CROP=$(python3 -c \
  "import yaml;c=yaml.safe_load(open('$REPO/config/sites/rusia/demo_view.yaml')).get('panel_crop');\
print(','.join(map(str,c)) if c else '')")
CROP_ARG=(); [ -n "$CROP" ] && CROP_ARG=(--panel-crop "$CROP")

echo "[componer] componiendo MP4… (panel_crop=${CROP:-ninguno})"
python3 scripts/demo_causal/compose_3panel.py \
  --real-shots "$REAL" --shadow-shots "$SHADOW/shots" \
  --frames-3d "$FRAMES3D" --ws-log "$WSLOG" \
  --out "$OUT" --title "$TITLE" "${CROP_ARG[@]}"
echo "[componer] LISTO: $OUT"

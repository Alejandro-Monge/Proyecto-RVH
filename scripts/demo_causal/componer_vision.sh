#!/usr/bin/env bash
# componer_vision.sh — VISION demo video (3 panels): VIDEO | VISION SHADOW | 3D.
# Same style as the MESSAGES one; reality = the camera VIDEO (vision DOES have video).
#
# (1) headless 3D render of the VISION ws_log (subsampled ~10Hz, scene_vision, with a
#     bit more zoom than MESSAGES); (2) render_split --frames-3d → 3-panel mp4. The
#     SUMO panel and the 3D come from the SAME causal vision reconstruction → coherent.
#
# Requires the VISION ws_log, generated with:
#   GRACE=0 "$PY" scripts/run_harness_demo_causal.py --3d --video tramo
set -euo pipefail
cd "$(dirname "$0")/../.."; REPO=$(pwd)
export SUMO_HOME=${SUMO_HOME:-/usr/share/sumo} DISPLAY=${DISPLAY:-:1}
PY="$REPO/.venv/bin/python3"    # the venv has the vision deps (supervision, etc.)

VIDEO=${VIDEO:-tramo}
WSLOG=${WSLOG:-$REPO/verify_out/demo_causal/web3d_$VIDEO/ws_log.jsonl}
FRAMES3D=${FRAMES3D:-$REPO/verify_out/demo_causal/web3d_$VIDEO/frames3d}
CAM=${CAM:-0.82}            # 3D camera distance factor (K<1 = more zoom than MESSAGES)
SUBSAMPLE=${SUBSAMPLE:-3}   # 1 out of every N ws_log steps (3 ≈ 10 Hz from ~29.6 Hz)
[ -f "$WSLOG" ] || { echo "falta $WSLOG — corre antes: run_harness_demo_causal.py --3d --video $VIDEO"; exit 1; }

py() { python3 -c "import yaml;print($1)"; }
HP=$(py "yaml.safe_load(open('$REPO/config/sites/rusia/web3d.yaml'))['vision_http_port']")
WP=$(py "yaml.safe_load(open('$REPO/config/sites/rusia/web3d.yaml'))['vision_ws_port']")
SCENE=$(py "yaml.safe_load(open('$REPO/config/sites/rusia/web3d.yaml'))['vision_scene']")
read -r W H <<< "$(python3 -c "import yaml;w=yaml.safe_load(open('$REPO/config/sites/rusia/demo_view.yaml'))['window_size'];print(w[0],w[1])")"

echo "[vision] webserver 3D (:$HP, escena $SCENE)…"
"$PY" -m ws_emitter.webserver --site rusia --scene "$SCENE" --http-port "$HP" --ws-port "$WP" \
  > /tmp/_web3d_vision.log 2>&1 &
WEB=$!; trap 'kill $WEB 2>/dev/null || true' EXIT; sleep 2

echo "[vision] render 3D headless (submuestreo 1/$SUBSAMPLE, cam=$CAM)…"
rm -rf "$FRAMES3D"; mkdir -p "$FRAMES3D"
NSTEPS=$(python3 -c "import json;print(sum(1 for l in open('$WSLOG') if l.strip() and json.loads(l).get('type')=='step'))")
IDX=$(python3 -c "print(','.join(str(i) for i in range(0,$NSTEPS,$SUBSAMPLE)))")
( cd "$REPO/scripts/demo_causal/recorder" && \
  node replay_ws3d.mjs --ws-log "$WSLOG" \
    --url "http://127.0.0.1:$HP/dev_replay.html?cam=$CAM" \
    --out "$FRAMES3D" --width "$W" --height "$H" --steps "$IDX" )

echo "[vision] render_split de 3 paneles (VIDEO | SOMBRA VISIÓN | 3D)…"
"$PY" scripts/run_harness_demo_causal.py --split --video "$VIDEO" --frames-3d "$FRAMES3D"
echo "[vision] LISTO: verify_out/demo_causal/demo_vision_3panel_$VIDEO.mp4"

#!/usr/bin/env bash
# grabar_realidad.sh — STEP 2 of the demo video: records the REALITY panel
# (GT2, the 70 real vehicles) with MANUAL FRAMING. It is recorded ONCE and reused
# in both videos (MESSAGES and VISION): reality is the same physical base.
#
# Same flow as grabar_sombra.sh: driver_replay opens its OWN PAUSED sumo-gui,
# YOU frame it by hand and press Enter here; the loop captures one shot per sim (sh_<ms>.png).
# Frame reality with the SAME view as the shadow so the panels line up.
#
# It MUST run in the foreground (this terminal): the Enter comes in via stdin.
#
# Usage:  scripts/demo_causal/grabar_realidad.sh
#         OUT=/output/path  scripts/demo_causal/grabar_realidad.sh
set -euo pipefail
cd "$(dirname "$0")/../.."            # -> implementacion/
REPO=$(pwd)
export SUMO_HOME=${SUMO_HOME:-/usr/share/sumo}
export DISPLAY=${DISPLAY:-:1}

OUT=${OUT:-$REPO/verify_out/demo_causal/realidad_gt2}
SCHEME=$REPO/config/sites/rusia/$(python3 -c \
  "import yaml;print(yaml.safe_load(open('$REPO/config/sites/rusia/demo_view.yaml'))['scheme_file'])")
WIN=$(python3 -c \
  "import yaml;w=yaml.safe_load(open('$REPO/config/sites/rusia/demo_view.yaml'))['window_size'];print(f'{w[0]},{w[1]}')")

mkdir -p "$OUT"
echo "[grabar_realidad] OUT=$OUT  scheme=$SCHEME  window=$WIN"
echo "[grabar_realidad] abriendo sumo-gui (realidad GT2). Encuadra y pulsa Enter aquí."
cd "$REPO/reconstruccion_veins"
python3 driver_replay.py --standalone-gui \
  --shot-dir "$OUT" --gui-settings-file "$SCHEME" --window-size "$WIN" \
  --manual-frame
echo "[grabar_realidad] LISTO: shots de realidad en $OUT"
echo "[grabar_realidad] siguiente: scripts/demo_causal/componer_mensajes.sh"

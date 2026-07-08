#!/usr/bin/env bash
# Step 2 gates: runs causal_player with --shot-dir TWICE (ON1, ON2) and compares.
set -u
REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
SP="${SP:-/tmp/gemelo_rvh_paso2}"
VIEW=$REPO/config/sites/rusia/sumo_demo_view.xml
export SUMO_HOME=/usr/share/sumo; export DISPLAY=:1
cd "$REPO/reconstruccion_causal"

run_on () {  # $1 = label (on1/on2)
  local tag=$1
  rm -rf "$SP/$tag"; mkdir -p "$SP/$tag/shots"
  python3 scripts/causal_player.py \
    --csv inputs/cert/mc_5Hz_P30_000.csv \
    --net inputs/intersection_rusia.net.xml \
    --out "$SP/$tag" \
    --delay-s 1.0 --period-s 0.2 --interp linear --guard-mode assert --staging-mode park \
    --gui --gui-settings-file "$VIEW" \
    --shot-dir "$SP/$tag/shots" --shot-boundary "-53.0,12.0,167.0,136.0" \
    --ws-log "$SP/$tag/ws_log.jsonl" > "$SP/$tag.log" 2>&1
  echo "$tag rc=$? shots=$(ls "$SP/$tag/shots" | wc -l)"
}

run_on on1
run_on on2

echo "=== GATE 1 (pasividad): validation_log OFF vs ON1 ==="
md5sum "$SP/off/results/validation_log.csv" "$SP/on1/results/validation_log.csv" 2>/dev/null
echo "=== pasividad extra: ws_log OFF vs ON1 (contenido del emit sin cambio) ==="
md5sum "$SP/off/ws_log.jsonl" "$SP/on1/ws_log.jsonl" 2>/dev/null
echo "=== GATE 2 (determinismo): shadow shots ON1 vs ON2 byte-idénticos ==="
python3 - "$SP/on1/shots" "$SP/on2/shots" <<'PY'
import sys, os, hashlib
d1, d2 = sys.argv[1], sys.argv[2]
f1 = sorted(os.listdir(d1)); f2 = sorted(os.listdir(d2))
print("count on1=%d on2=%d mismos-nombres=%s" % (len(f1), len(f2), f1 == f2))
def md5(p): return hashlib.md5(open(p, 'rb').read()).hexdigest()
common = [f for f in f1 if f in set(f2)]
diff = [f for f in common if md5(os.path.join(d1, f)) != md5(os.path.join(d2, f))]
print("PNG byte-idénticos: %d/%d | difieren: %d" % (len(common) - len(diff), len(common), len(diff)))
if diff:
    print("primeros diferentes:", diff[:5])
PY
echo "=== DONE ==="

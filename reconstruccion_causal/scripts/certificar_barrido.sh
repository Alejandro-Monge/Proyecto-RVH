#!/usr/bin/env bash
# certificar_barrido.sh — causal certification matrix over the MC sweep.
#
# It runs the causal reconstructor in ASSERT mode (interp linear + HELD) over
# 3 frequencies x 4 loss levels (1 rep per combo) and reports PASS/FAIL
# with the guard's maximum lookahead. If a combo triggers
# CausalityViolation it is a FINDING: it is reported, not patched.
#
# Usage: ./scripts/certificar_barrido.sh [P_JOBS]   (default 3 in parallel)
set -euo pipefail
cd "$(dirname "$0")/.."

MC=../via_b_runtime/RusiaV2IViaB/results/montecarlo
NET=inputs/intersection_rusia.net.xml
JOBS=${1:-3}
mkdir -p inputs/cert runs/cert_matrix

combos=()
for f in 1Hz 2Hz 5Hz; do
  for p in P00 P25 P40 P50; do
    combos+=("$f:$p")
  done
done

period_of() { case "$1" in 1Hz) echo 1.0;; 2Hz) echo 0.5;; 5Hz) echo 0.2;; esac; }

run_combo() {
  local combo=$1 f p src dst per out
  f=${combo%%:*}; p=${combo##*:}
  src="$MC/$f/$p/rep000.csv"; [ -f "$src" ] || src="$MC/$f/$p/base.csv"
  dst="inputs/cert/mc_${f}_${p}.csv"
  per=$(period_of "$f")
  out="runs/cert_matrix/${f}_${p}"
  [ -f "$dst" ] || python3 scripts/preparar_entrada.py --src "$src" --dst "$dst" >/dev/null
  rm -rf "$out"
  if python3 scripts/causal_player.py --csv "$dst" --net "$NET" --out "$out" \
      --delay-s 1.0 --period-s "$per" --interp linear --guard-mode assert \
      >"$out.log" 2>&1; then
    echo "$combo OK"
  else
    echo "$combo FAIL(exit=$?)"
  fi
}
export -f run_combo period_of
export MC NET

printf '%s\n' "${combos[@]}" | xargs -P "$JOBS" -I{} bash -c 'run_combo "$@"' _ {}
echo "== corridas terminadas; resumen: =="
python3 - <<'EOF'
import json, os
print(f"{'combo':10s} {'guardia':8s} {'viol':>5s} {'lookahead_s':>12s} {'estados':>8s} {'held':>6s} {'retiros':>8s} {'reacq':>6s}")
for f in ["1Hz", "2Hz", "5Hz"]:
    for p in ["P00", "P25", "P40", "P50"]:
        path = f"runs/cert_matrix/{f}_{p}/results/summary.json"
        if not os.path.exists(path):
            print(f"{f}/{p:6s} SIN SUMMARY (crash antes de escribir salidas)")
            continue
        s = json.load(open(path))
        g = s["guardia"]
        print(f"{f+'/'+p:10s} {g['status']:8s} {g['violations']:5d} "
              f"{g['max_lookahead_s']:12.6f} {s['estados_aplicados']:8d} "
              f"{s.get('held_frames_causal',0):6d} {s.get('retiros_totales',0):8d} "
              f"{s.get('reacquisiciones',0):6d}"
              + ("  ABORT: " + s["guardia_abort"][:80] if s.get("guardia_abort") else ""))
EOF

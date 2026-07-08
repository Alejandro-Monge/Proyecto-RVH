#!/usr/bin/env bash
# barrido_denso.sh [piloto|full] [NPAR] — runs the causal reconstructor (assert)
# over the messages CSVs from montecarlo/ (raw IDs, no shim) dumping the
# DENSE CSV per run to resultados_densos/<freq>/P<lv>/rep###.csv.gz
# (a NEW path, mirror of the sweep; montecarlo/ and existing results intact).
# It saves a control summary.json per run in resultados_densos/_ctl/ and a
# timing log per run. The manifest (md5+counts) is written by
# manifiesto_denso.py at the end.
#
#   piloto: 3 freq x {P00,P10,P25,P40,P50} x reps 000-002 (P00 = base) = 39
#   full:   3 freq x {P00 + P05..P50 x 100 reps} = 3003
set -uo pipefail
MODE=${1:-piloto}; NPAR=${2:-16}
cd "$(dirname "$0")/.."
MC=../via_b_runtime/RusiaV2IViaB/results/montecarlo
DENSO=../via_b_runtime/RusiaV2IViaB/results/resultados_densos
NET=inputs/intersection_rusia.net.xml
mkdir -p "$DENSO/_ctl"
TLOG="$DENSO/_ctl/tiempos_${MODE}.log"
: > "$TLOG"

period_of() { case "$1" in 1Hz) echo 1.0;; 2Hz) echo 0.5;; 5Hz) echo 0.2;; esac; }

combos() {
  if [ "$MODE" = piloto ]; then
    for f in 1Hz 2Hz 5Hz; do
      echo "$f:P00:base"
      for L in P10 P25 P40 P50; do for r in 000 001 002; do echo "$f:$L:rep$r"; done; done
    done
  else
    for f in 1Hz 2Hz 5Hz; do
      echo "$f:P00:base"
      for L in P05 P10 P15 P20 P25 P30 P35 P40 P45 P50; do
        for r in $(seq -f "%03g" 0 99); do echo "$f:$L:rep$r"; done
      done
    done
  fi
}

run_one() {
  local combo=$1 f L rep src per out dense t0 t1 rc
  IFS=: read f L rep <<< "$combo"
  src="$MC/$f/$L/$rep.csv"
  dense="$DENSO/$f/$L/$rep.csv.gz"
  out="$DENSO/_ctl/${f}_${L}_${rep}"
  [ -s "$dense" ] && { echo "SKIP $combo" >> "$TLOG"; return 0; }
  [ -f "$src" ] || { echo "NOSRC $combo" >> "$TLOG"; return 1; }
  per=$(period_of "$f")
  t0=$(date +%s.%N)
  if python3 scripts/causal_player.py --csv "$src" --net "$NET" --out "$out" \
      --delay-s 1.0 --period-s "$per" --interp linear --guard-mode assert \
      --dense-out "$dense" > "$out.log" 2>&1; then rc=OK; else rc="FAIL($?)"; fi
  t1=$(date +%s.%N)
  # light control: summary.json and log are kept; the per-run validation_log
  # duplicates the dense dump and is heavy: it is removed in full (kept in piloto)
  if [ "$MODE" = full ]; then rm -f "$out/results/validation_log.csv" "$out/results/crossing_times.csv"; fi
  echo "$combo $rc $(echo "$t1 $t0" | awk '{printf "%.1f", $1-$2}')s" >> "$TLOG"
  [ "$rc" = OK ] || echo "FALLO: $combo ($rc)"
}
export -f run_one period_of
export MC DENSO NET TLOG MODE

T0=$(date +%s)
combos | xargs -P "$NPAR" -I{} bash -c 'run_one "$@"' _ {}
echo "DENSO-DONE modo=$MODE t=$(( $(date +%s) - T0 ))s ok=$(grep -c ' OK ' $TLOG) fallos=$(grep -vc ' OK \|SKIP' $TLOG || true)"

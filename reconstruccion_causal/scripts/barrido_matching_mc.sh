#!/bin/bash
# barrido_matching_mc.sh — Section 6 matching + Section 7 metrics over the 3003 MC runs.
#
# Per run: causal_player standalone (dense dump to tmp, DELETED at the end; it is
# deterministically regenerable) -> matching_mensajes --no-sweep --freq (per-
# frequency window from config, calibrated P00∩P50+0.2) -> only the matching
# artifacts remain (~40 KB). Resumable: skip if resumen_matching.json exists. Final
# census BY FILES (lesson from the broken counter of the original MC sweep).
# Parallel: N workers (default 12; 32-core host, no docker or Veins).
set -uo pipefail
RAIZ="$(cd "$(dirname "$0")/../.." && pwd)"
MC="$RAIZ/via_b_runtime/RusiaV2IViaB/results/montecarlo"
RC="$RAIZ/reconstruccion_causal"
OUTBASE="$RC/runs/matching_mc"
N=${N:-12}
FILTER=${FILTER:-}          # e.g. FILTER="1Hz/P05" for partial smoke

worker() {
    csv="$1"
    rel="${csv#"$MC"/}"                        # 1Hz/P05/rep000.csv
    freq="${rel%%/*}"
    resto="${rel#*/}"; lvl="${resto%%/*}"
    rep="$(basename "$csv" .csv)"
    out="$OUTBASE/$freq/$lvl/$rep"
    [ -f "$out/resumen_matching.json" ] && return 0
    case "$freq" in 1Hz) per=1.0;; 2Hz) per=0.5;; 5Hz) per=0.2;; *) echo "freq? $rel" >>"$OUTBASE/fails.log"; return 1;; esac
    tmpd="$(mktemp -d)"
    if ! python3 "$RC/scripts/causal_player.py" --csv "$csv" \
            --net "$RC/inputs/intersection_rusia.net.xml" --out "$tmpd/run" \
            --delay-s 1.0 --period-s "$per" --interp linear --guard-mode assert \
            --dense-out "$tmpd/denso.csv.gz" >/dev/null 2>&1; then
        echo "PLAYER_FAIL $rel" >> "$OUTBASE/fails.log"; rm -rf "$tmpd"; return 1
    fi
    if ! "$RAIZ/.venv/bin/python" "$RC/scripts/matching_mensajes.py" \
            --dense "$tmpd/denso.csv.gz" --out "$out" --freq "$freq" --no-sweep \
            >/dev/null 2>&1; then
        echo "MATCH_FAIL $rel" >> "$OUTBASE/fails.log"; rm -rf "$tmpd"; return 1
    fi
    rm -rf "$tmpd"
}
export -f worker
export MC RC RAIZ OUTBASE

mkdir -p "$OUTBASE"
find "$MC" -name "*.csv" | { [ -n "$FILTER" ] && grep "$FILTER" || cat; } | sort > "$OUTBASE/_jobs.txt"
TOTAL=$(wc -l < "$OUTBASE/_jobs.txt")
echo "[barrido_matching] $TOTAL corridas, N=$N workers"
xargs -a "$OUTBASE/_jobs.txt" -P "$N" -I{} bash -c 'worker "$@"' _ {}

HECHAS=$(find "$OUTBASE" -name resumen_matching.json | wc -l)
echo "[censo por archivos] resumen_matching.json: $HECHAS / $TOTAL"
[ -f "$OUTBASE/fails.log" ] && { echo "FALLOS:"; sort "$OUTBASE/fails.log" | uniq -c | sort -rn | head; }
[ "$HECHAS" -eq "$TOTAL" ] && echo "BARRIDO COMPLETO"

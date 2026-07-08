// hud_core — PURE aggregators of the client's OWN metrics (no DOM, no THREE): the ONLY
// thing the client is allowed to "compute" are aggregates of its network/render
// measurements (WS→render latency, coalescing). Every domain number comes from the
// payload and is drawn as-is (HUD contract, docs/modules/ws_hud.md: "a missing number
// => missing from the CONTRACT, not a client-side computation").
//
// PERCENTILE: nearest-rank with THE SAME convention as the player
// (causal_player.py, pct(a, p) = a[min(len-1, int(p*len))] over the sorted list) —
// p50/p95 comparable server/client, not two definitions. Consequence pinned in tests:
// the median of [1,2,3,4] is 3 (nearest-rank), not 2.5 (arithmetic mean) — same as in
// the player's summary.

export function pct(sortedAsc, p) {
  if (!sortedAsc.length) return null;
  return sortedAsc[Math.min(sortedAsc.length - 1,
                            Math.floor(p * sortedAsc.length))];
}

// Reception coherence — TWO CASES (fix 2026-07-05, diagnosed with a late connection
// reproduced: firstSeq 325 vs first seq 4, received == exact window, wireGaps 0 → the
// old identity gave a false MISMATCH):
//  - COMPLETA (the client was there from the stream's first step): the exact identity
//    of case 7, received == steps − dropped.
//  - TARDIA (entered the "now" — retained hello + ephemeral steps, approved design):
//    the observed window is [firstSeq..seq_final]; missing_in_window =
//    (seq_final − firstSeq + 1) − received, and the client is HEALTHY if missing ≤
//    declared dropped (real loss = gaps WITHIN the window, not what came before
//    entering).
// stream's first seq: derived from what is DECLARED in the end (seq_final and steps;
// the log has no gaps — property 2.3): first = seq_final − steps + 1. The AUTOMATED
// case 7 (test_stream_protocol) does not change: there the client connects from the
// start and requires exact equality.
export function coherenceVerdict(stats, end) {
  const dropped = end.totals.dropped;
  const firstStream = end.seq_final - end.totals.steps + 1;
  if (stats.firstSeq === null || stats.firstSeq === undefined) {
    return { caso: 'SIN_STEPS', ok: stats.received === 0, lateMiss: null,
             missing: null, esperado: 0 };
  }
  const lateMiss = Math.max(0, stats.firstSeq - firstStream);
  const ventana = end.seq_final - stats.firstSeq + 1;
  const missing = ventana - stats.received;
  return {
    caso: lateMiss === 0 ? 'COMPLETA' : 'TARDIA',
    lateMiss,                          // informational: lost by entering late
    missing,                           // missing WITHIN the window
    esperado: ventana - Math.min(missing, dropped),
    ok: missing <= dropped,            // undeclared real loss => MISMATCH
  };
}

export class ClientMetrics {
  constructor() {
    this._lat = [];            // ms samples, only from PAINTED steps
    this.skippedTotal = 0;     // client coalescing (received and not painted)
  }

  addLatency(ms) { this._lat.push(ms); }

  addSkipped(n) { this.skippedTotal += n; }

  snapshot() {
    const s = [...this._lat].sort((a, b) => a - b);   // does not mutate the state
    return {
      n: s.length,
      median_ms: pct(s, 0.5),
      p95_ms: pct(s, 0.95),
      max_ms: s.length ? s[s.length - 1] : null,      // null, never NaN
      skippedTotal: this.skippedTotal,
    };
  }
}

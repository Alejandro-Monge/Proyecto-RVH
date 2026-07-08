// HEADLESS verification of hud_core.js (node >=22; orchestrated by pytest).
// Pin of the nearest-rank convention AGAINST THE PLAYER FORMULA
// (causal_player.py: pct(a, p) = a[min(len-1, int(p*len))] over the sorted array).
import assert from 'node:assert/strict';
import { ClientMetrics, pct } from '../ws_emitter/web/js/hud_core.js';

let n = 0;
const ok = (c, m) => { assert.ok(c, m); n++; };

// player formula replicated INDEPENDENTLY (the convention pin)
const playerPct = (sorted, p) =>
  sorted.length ? sorted[Math.min(sorted.length - 1, Math.trunc(p * sorted.length))]
                : null;

// 1) empty -> null, never NaN
let m = new ClientMetrics();
let s = m.snapshot();
ok(s.n === 0 && s.median_ms === null && s.p95_ms === null && s.max_ms === null,
   'vacío -> null (no NaN)');
ok(!Number.isNaN(s.median_ms), 'null, no NaN');

// 2) a single element
m = new ClientMetrics(); m.addLatency(5);
s = m.snapshot();
ok(s.median_ms === 5 && s.p95_ms === 5 && s.max_ms === 5, 'singleton');

// 3) odd [1..5]: median a[int(0.5*5)]=a[2]=3; p95 a[min(4,int(4.75))]=a[4]=5
m = new ClientMetrics();
for (const v of [3, 1, 5, 2, 4]) m.addLatency(v);   // deliberately unsorted
s = m.snapshot();
ok(s.median_ms === 3 && s.p95_ms === 5 && s.max_ms === 5, 'impar nearest-rank');

// 4) even [1..4]: NEAREST-RANK median = a[2] = 3 (NOT 2.5, the arithmetic mean):
// the SAME convention as the player summaries — comparable server/client
m = new ClientMetrics();
for (const v of [4, 2, 1, 3]) m.addLatency(v);
s = m.snapshot();
ok(s.median_ms === 3, 'par: nearest-rank (3), no media (2.5)');

// 5) pin against the player formula on a large deterministic array
const arr = [];
let seed = 20260705;
for (let k = 0; k < 997; k++) {           // prime, non-round size
  seed = (seed * 1103515245 + 12345) % 2147483648;
  arr.push(seed % 10000 / 10);
}
m = new ClientMetrics();
for (const v of arr) m.addLatency(v);
s = m.snapshot();
const sorted = [...arr].sort((a, b) => a - b);
ok(s.median_ms === playerPct(sorted, 0.5), 'p50 == fórmula del player');
ok(s.p95_ms === playerPct(sorted, 0.95), 'p95 == fórmula del player');
ok(pct(sorted, 0.5) === playerPct(sorted, 0.5), 'pct exportada == player');

// 6) accumulated skipped + snapshot idempotency
m = new ClientMetrics();
m.addSkipped(3); m.addSkipped(0); m.addSkipped(7);
m.addLatency(1); m.addLatency(2);
const s1 = m.snapshot(), s2 = m.snapshot();
ok(s1.skippedTotal === 10, 'skipped acumulado');
assert.deepEqual(s1, s2); n++;            // idempotent
m.addLatency(0.5);                        // and still alive after snapshot
ok(m.snapshot().n === 3, 'snapshot no congela el acumulador');

// 7) coherence over TWO CASES (fix 2026-07-05)
import('../ws_emitter/web/js/hud_core.js').then(() => {});
const { coherenceVerdict } = await import('../ws_emitter/web/js/hud_core.js');
const END = (seqFinal, steps, dropped) =>
  ({ seq_final: seqFinal, totals: { steps, dropped } });

// exact COMPLETA case (the identity of automated case 7, intact)
let v = coherenceVerdict({ firstSeq: 4, received: 1864 }, END(1867, 1864, 0));
ok(v.caso === 'COMPLETA' && v.ok && v.missing === 0, 'completa exacta OK');
// COMPLETA with declared drops
v = coherenceVerdict({ firstSeq: 4, received: 1800 }, END(1867, 1864, 64));
ok(v.caso === 'COMPLETA' && v.ok && v.missing === 64, 'completa con drops == declarado');
// healthy TARDIA — the numbers from the reproduced DIAGNOSIS (firstSeq 325)
v = coherenceVerdict({ firstSeq: 325, received: 1543 }, END(1867, 1864, 0));
ok(v.caso === 'TARDIA' && v.ok && v.missing === 0 && v.lateMiss === 321,
   'tardía sana (repro 325): OK, no DESAJUSTE');
// healthy TARDIA — the demo numbers (received 1626)
v = coherenceVerdict({ firstSeq: 242, received: 1626 }, END(1867, 1864, 0));
ok(v.caso === 'TARDIA' && v.ok && v.missing === 0 && v.lateMiss === 238,
   'tardía sana (demo Israel 1626): OK');
// TARDIA with REAL undeclared loss -> mismatch
v = coherenceVerdict({ firstSeq: 242, received: 1600 }, END(1867, 1864, 0));
ok(v.caso === 'TARDIA' && !v.ok && v.missing === 26, 'pérdida real => DESAJUSTE');
// no steps received
v = coherenceVerdict({ firstSeq: null, received: 0 }, END(1867, 1864, 0));
ok(v.caso === 'SIN_STEPS' && v.ok, 'sin steps: ok trivial');

console.log(`OK: ${n} verificaciones de hud_core`);

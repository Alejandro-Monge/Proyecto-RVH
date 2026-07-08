# Module — Metrics HUD (`ws_emitter/web/js/hud.js`, `hud_core.js`)

Metrics overlay of the messages client (profile `messages`): it draws, it does not compute. Its
axis is the closing of the loop between payload and render, with each number shown next to its
origin.

## HUD contract

Rule: if the HUD needed a number not in this table, it is a gap in the
payload contract, not a license to compute it in the client. The only things computed
are aggregates of the browser's own network and render metrics.

| Element | Origin | Client computation? |
|---|---|---|
| State + reason | `stream.state/reason` | no |
| `t_sim`, `seq` | `step` | no |
| `site/dt/layers/schema` + occupancy semantics | `hello` | no |
| **Approach/lane occupancy (ocupación·detenidos)** | `step.occupancy.per_lane` (server-side, occupancy_core, audited layer 2) | **no — cell by cell** |
| Guard (accesses/violations/lookahead) | `step.guard` | no |
| `wireGaps` (SERVER loss) | `stream.stats()` | no |
| `skipped` (CLIENT coalescing) | `takeLatest().skipped` | sum (own metric) |
| Ignored by layers / unknown vclass | `entityLayer.counters` | no |
| **WS→render (n/median/p95/max)** | own samples `t_render_apply − t_recv` (one clock: `performance.now`), ONLY painted steps; labeled "up to render_apply (there is no pixel callback in WebGL)" | **yes — the only one allowed** (hud_core) |
| Verdict at `end` + totals | `end.sonda.status`, `end.totals` | no |
| Reception coherence — **TWO CASES** (`hud_core.coherenceVerdict`) | own accounting vs what the `end` declares | comparison of own metrics (see the Coherence section) |
| No `end` | banner "run without verdict" | no |
| Provenance legend | LIVES HERE (a single place) | no |
| JSON export | dump of everything above | serialization |

## Reception coherence

A client with a late connection enters the "now" (hello retained + ephemeral steps): it receives
exactly its window from the seq at which it entered (`wireGaps 0`), not the complete stream. The
identity of case 7 (`received == steps − dropped`) only holds for a complete connection; applied
to a late connection it produces a false positive. That is why reception
coherence is evaluated in two cases.

Two-case formula (`hud_core.coherenceVerdict`, pure and tested):
`first_seq_stream = end.seq_final − end.totals.steps + 1` (derived from what is declared; the log
is gapless by construction); `lateMiss = firstSeq − first_seq_stream` (informative, not
a loss); `missing_in_window = (end.seq_final − firstSeq + 1) − received`; OK if
`missing_in_window ≤ dropped`, i.e., a mismatch only from real undeclared loss.
Labels: "complete from the start" / "late connection (entered at seq X; N before entering,
they do not count as loss)".

The automated case 7 does not change (`test_stream_protocol`): there the client connects from the
start (verified, `firstSeq == first seq of the log`) and the exact equality is still required. The
general formula contains it (lateMiss = 0 ⇒ missing = steps − dropped − received). The
`dev_stream.html` scaffolding keeps the complete-connection line with a note that refers here.

Legitimate case `vehiculos=0` with occupancy 0/0 at the end: the server-side log can show a
maximum of 23 entities midway through the stream (mean 11.8), with the last non-zero occupancy at
`t_sim` 174.0 of 186.7 and empty final steps: the intersection emptied out as the run ended.
It is not a render failure.

Separator of the occupancy table: the `ocupación/detenidos` cell uses "/" and an explicit header
(a "·" would read as a decimal).

## Percentiles

`pct(sorted, p) = sorted[min(len − 1, int(p·len))]`, the same formula as the `causal_player.py`
summary: p50/p95 comparable between server and client, not two definitions.
Consequence fixed in test: the median of `[1,2,3,4]` is **3** (nearest-rank), not 2.5.

## Verifications

`tests/hud_core_check.mjs` (node, orchestrated by pytest): empty → null (no NaN); singleton;
odd/even with exact values; comparison against the player's formula, replicated independently
over 997 deterministic samples; accumulated skipped; idempotent and non-freezing snapshot. It adds
the import graph (`dev_hud.html` picked up by the glob) and `node --check`.

## Visual validation

Commands:
```
A: python3 -m http.server 8300 --directory ws_emitter/web
B: cd reconstruccion_causal && ../.venv/bin/python scripts/causal_player.py \
     --csv inputs/cert/mc_2Hz_P00.csv --net inputs/intersection_rusia.net.xml \
     --out /tmp/ws3d --period-s 0.5 --occupancy-out /tmp/ws3d/occ.csv \
     --ws-log /tmp/ws3d/ws.jsonl --ws-port 8931 --ws-open-grace 15 --ws-realtime
Browser: http://localhost:8300/dev_hud.html
```

1. Overlay legible without covering the scene.
2. **The occupancy table (server-side) matches the drawn vehicles**: visual closing of the payload↔render loop.
3. The two separate losses and the WS→render latency, understandable.
4. At the end: probe verdict and coherence `received == steps−dropped`.
5. Provenance legend in its place (a single spot).

`dev_hud` reuses roadway, stream and entities; the provenance legend lives in the HUD (a
single place).

## References

- `ws_payload_contract.md`: the payload contract that originates each HUD number.
- `ws_stream.md`: the WS consumer that provides `stats()` and `wireGaps`.
- `ws_entities.md`: the counters of ignored entities and the shared provenance legend.
- `occupancy_mensajes.md`: the server-side `per_lane` occupancy that the table shows.

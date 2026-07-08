# Module — WS consumer of the v3 contract (`ws_emitter/web/js/stream.js`)

State machine of the client's WS protocol (messages profile). It does not draw, does not compute
domain, does not reorder (WS is FIFO per connection, RFC 6455), does not interpolate and does not compare md5. It is
pure protocol and is verified by code, not by visual observation (the visual validation
belongs to entities.js). The verification runs in node v22.23.0 with global WebSocket,
executing the real machine of stream.js (`tests/stream_harness.mjs`, the same file that
serves the client, without port or mock), orchestrated by pytest (`tests/test_stream_protocol.py`)
against the real emitter and `dev_ws_negativo`. The emitter→client pipeline is tested end to
end in case 1.

## State machine

`WAIT_HELLO → STREAMING → ENDED | SIN_VEREDICTO | RECHAZADO` (terminal). step/end before
hello → RECHAZADO; `schema_version != 3.0.0` → RECHAZADO; duplicate hello / invalid JSON /
unknown type → RECHAZADO; close without end → SIN_VEREDICTO (never an implicit PASS); messages
after terminal → ignored and counted (`postTerminal`). `reject(reason)` is the only external path
to RECHAZADO: main.js compares `hello.net_md5` against the scene and, if it fails, transitions by that
path (symmetry with net_scene, which exposes its `netMd5`; the state lives in a single place).

## Input and output

`new StreamClient(url)` (URL by parameter; the wiring to web3d.yaml arrives with main.js/--3d) ·
`connect()` · `state/hello(frozen)/end(frozen)/reason` · `reject(reason)` · `takeLatest() →
{step, tRecv, skipped}|null` · `stats() → {received, firstSeq, lastSeq, wireGaps,
postTerminal}` · callbacks `onStateChange/onHello/onEnd`. "last received" buffer (last writer
wins, temporal anchoring); `tRecv = performance.now()` in onmessage (the client-side mark for
the hud). Real wire detail: the emitter sends binary frames →
`binaryType='arraybuffer'` plus TextDecoder (a client that assumed text would reject everything).

## The two separate losses

- **`wireGaps`** — seq gaps in what is received = server drops (the `dropped` that the
  end declares; the canonical log does not have them, the wire does).
- **`skipped`** (by `takeLatest`) — received items that the painting did not consume = client
  coalescing.
- **Coherence identity (case 7):** with hello and end received, `received ==
  end.totals.steps − end.totals.dropped`, exact: TCP delivers everything sent before the ordered
  close; `dropped` counts what was never sent. The scaffolding verifies it at the end
  (`OK`/`DESAJUSTE` visible).

## Scaffolding

- `dev_stream.html` — observable overlay: transition log with timestamp, hello
  summary, live counters (consumption at rAF to make the coalescing visible), verdict plus
  coherence identity. `?url=` (default `ws://127.0.0.1:8931`), `?expect=<md5>` (simulates
  the main.js comparison → `reject()`).
- `ws_emitter/dev_ws_negativo.py` — negative control: serves agreed broken sequences
  (step-primero, schema-malo, hello-doble, json-roto, cierre-sin-end) over a real socket.
  Verified by pytest (`tests/test_dev_ws_negativo.py`, 5/5): the scaffolding server can also
  lie, so what it serves is verified.

## hello delivery: retained session state

The `hello` is immutable session state, retained by the server since its construction; each
connection receives it in its on-connect handler (websockets 16.0 doc: the handler receives a
`ServerConnection` per client). It cannot be emitted only once with `broadcast()`, which only
reaches those connected at that instant (16.0 doc, topics/broadcast: the pattern is a set of
connections registered): a client that connects after startup would receive "step before
hello". The hello delivery is synchronous plus registration in the same block without await (atomic in
the event loop of a single thread), by a single path (never broadcast or queue): thus there is no
duplicate race nor a drain of the first step slipping in during a yield.

**Declared semantics:** ephemeral steps (whoever enters midway receives the "now", the last
step wins: it is replay, you enter the now); the `end` with best-effort delivery (put with timeout ≤2 s,
post-simulation, zero impact on dynamics; without this, with the queue saturated at close the
verdict would fall off the wire); whoever enters after the end receives hello and close → SIN_VEREDICTO
for that viewer.

**Timing characteristics:** the complete run lasts ~4.5–10 s of wall (~1400 streaming steps/s), hence
`--ws-open-grace` in the player (wait post-hello and pre-loop so viewers can connect; pure wall
time, passivity verified) and the synchronization of tests by log content, not by sleeps; and
`--ws-stall-drain` delays per drained item (sleeping before the get would accumulate sleeps with
the queue empty).

## Table of verified cases

The seven cases are verified in an automated way; 7/7 pass.

| # | Case | How it was produced (pytest) | Result |
|---|---|---|---|
| 1 | Happy path with LATE client | real player; harness connects with ≥300 steps already emitted | **PASS**: hello→STREAMING→ENDED `sonda=PASS`; `firstSeq > first seq of the log` (enters the NOW, without re-emission) |
| 2 | Close without end | SIGKILL to the player with ≥150 steps emitted and client in STREAMING | **PASS**: SIN_VEREDICTO("…sin end"), received>0, end=null |
| 3 | step without hello | dev_ws_negativo `step-primero` | **PASS**: RECHAZADO("antes de hello") |
| 4 | schema mismatch | `schema-malo` (2.9.9) | **PASS**: RECHAZADO("schema_version…") |
| 5 | net_md5 mismatch | real hello from the negative + `expect=deadbeef…` | **PASS**: RECHAZADO via `reject()` |
| 6 | double hello / broken JSON | `hello-doble` / `json-roto` | **PASS**: RECHAZADO with reason |
| 7 | wireGaps + IDENTITY + passivity | adversarial calibrated to measured rates (`queue 4, stall 2 ms/item, grace 3 s`) with client registered BEFORE the first step (verified `firstSeq == first seq of the log`) | **PASS**: wireGaps>0, **`received == end.totals.steps − end.totals.dropped` EXACT**, and `validation_log.csv` == baseline (`1c49a6c0…`) — passivity institutionalized in the test |

Smoke tests: `test_web_imports.py` (all the `*.html`) + `test_dev_ws_negativo.py` (agreed
sequences) + `node --check`. Full WS suite: 59/59. The scaffolding
`dev_stream.html`/`dev_ws_negativo.py` remain as manual inspection tools.

## References

- `docs/modules/ws_payload_contract.md`: the v3 contract that this machine consumes.
- `docs/modules/ws_emitter_impl.md`: the real emitter against which the protocol is verified.
- `docs/modules/ws_net_scene.md`: net_scene, which exposes the `netMd5` compared against the hello.
- `docs/modules/ws_entities.md`: the entities layer where the visual validation enters.

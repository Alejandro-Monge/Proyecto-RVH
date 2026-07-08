# WebSocket message emitter (`ws_emitter/emitter.py`, `server.py`)

Emitter of the `shadow=messages` profile of the v3.0.0 contract (`ws_payload_contract.md`, pin `e25a04be…`, architecture B). It chains the components: `DecidedTickView` → `step_message()` → `canonical_bytes()` → canonical log → queue → `websockets` thread with `broadcast()`. Dependency: `websockets==16.0` (pinned in `requirements.txt`).

## Components

- **`emitter.py`, deterministic side (player thread):**
  - `MessagesEmitter(log_path, hello_fields, ws_host, ws_port, queue_max, stall_drain_s)`. `ws_port=0` = no network (log only). Rule: no network without a log.
  - `emit_hello()` before the first `simulationStep` (first line of the log); `emit_step(view)` with invariable order: map → canonicalize → write log (`flush` per line: a crash does not truncate what the prefix audits) → `put_nowait`; `emit_end(guard_final, status, detail)` after the last step (last line); `close()` → report `{steps, dropped, emit_errors, server}` (goes to the player's `summary.json`, key `ws_emitter`).
  - `step_message(view)` is the pure view→dict function of the contract (step and entities sections).
  - Emitter errors: counted (`emit_errors`), they never stop the player.
- **`server.py`, emitter thread:** `WsBroadcastServer(host, port, queue)` is a daemon thread with its own asyncio event loop; the handler only registers connections; `notify()` bridges with `loop.call_soon_threadsafe` (official Python doc, asyncio-dev: "To schedule a callback from another OS thread, the loop.call_soon_threadsafe() method should be used"); `_drain()` runs in the loop: takes bytes from the queue, `wire_frame(bytes, time.monotonic())` (injection of `t_emit` over a copy) and `broadcast()`. The thread does not import traci nor receive references to the player. `stall_drain_s` is a diagnostic delay of the drain (adversarial control, in the line of `--leak-probe`).
- **`probe_client.py`:** diagnostic: verification of reception, order and `t_emit`, and passivity adversary (`--no-consume`, `--slow-s`). It is not the 3D client.

## Verifications against documentation

Against the installed version, not a generic one:

- **websockets 16.0** (reported by `websockets.__version__` after the pin). `broadcast(connections, message, raise_exceptions=False)` (introspection with `inspect.signature`). Docstring of the installed version: "pushes the message synchronously to all connections even if their write buffers are overflowing. There's no backpressure". Versioned doc (readthedocs /en/16.0/, topics/broadcast): the laggard "eventually … reaches the limit defined by ping_timeout and websockets terminates the connection"; the recommended implementation is the asyncio one (`websockets.asyncio.server.serve/broadcast`), the one used; the legacy one was left behind since the API change of the 13/14 series.
- **Thread bridge:** `call_soon_threadsafe` (quoted above; asyncio is not thread-safe: "Almost all asyncio objects are not thread safe").

## Heading conversion

Official source (SUMO TraCI/Change_Vehicle_State, the moveToXY angle parameter): "The angle value is assumed to be in navigational degrees (between 0 and 360 with 0 at the top, going clockwise)", and after the mapping the vehicle assumes that angle for drawing and outputs; `VAR_ANGLE 0x43` returns "the angle of the named vehicle within the last step [°]" (Vehicle Value Retrieval; the row does not re-declare the convention, which lives in moveToXY and flows to the outputs). The contract asks for radians, 0 = +x (east), counterclockwise, [0, 2π). Formula (single conversion point): `rad = radians((90 − deg) mod 360)`. Table fixed in `tests/test_ws_emitter.py`: N(0°)→π/2, E(90°)→0, S(180°)→3π/2, O(270°)→π. Unknown origin label → error (no orientation is emitted without a source).

## Player flags

`--ws-log PATH` (activates view plus emitter; the truth on disk), `--ws-port N` (network, requires `--ws-log`), `--ws-queue-max` (default 1024), `--ws-stall-drain` (diagnostic), `--ws-site` (site_id of the hello). The `hello` carries real md5s (messages csv, net, occupancy.yaml and lane_map if OCC), `dt = core.TICK`, `frame_geometry: null` (messages profile, no video, does not touch the fixed contract) and `vclass_dims` from `SHADOW_DIMS`.

## Verification results

Run `mc_2Hz_P00`, OCC, assert guard:

1. **Adversarial passivity (hard criterion):** minimal queue (2) + drain with 50 ms of diagnostic delay + probe_client connected without consuming → **dropped = 1752 of 1864 steps** (94 % of the network discarded, `sent = 114`) and even so `validation_log.csv`, `crossing_times.csv` and `occ.csv` **byte-identical** to baseline 2.1 (`1c49a6c0…` / `68b5adee…` / `b5adc768…`). Guard PASS, 22000 states. Architecture B confirmed; C was not necessary.
2. **Log determinism:** two runs of the same artifact (`--ws-log` only) → identical md5 (`f53c32d0…`). Moreover, adversarial vs deterministic: hello plus 1864 steps byte-identical; only the `end` line differs in `totals.dropped` (1751 vs 0), the declaration of the network loss of each run. Property declared for the prefix audit: the prefix comparison audits `hello` plus complete `steps`; from the `end` it excludes the network counters (`totals.dropped`), which are thread state, not reconstruction state.
3. **`t_emit` out of the log:** 0 occurrences (grep) in both logs; the wire does carry it (probe and round-trip test: wire without `t_emit` re-canonicalized == log line, byte for byte).
4. **hello/end order:** first line `hello` (schema 3.0.0), 1864 steps with strictly increasing seq, last line `end` with verdict (`PASS`); and in code: `emit_hello()` before the loop, `emit_end()` in the `finally`.

## Delivery semantics by message type

The hello is retained:

- **hello, retained:** immutable session state fixed at server construction; each client receives it in its on-connect handler, with synchronous delivery (`broadcast({ws},…)`) plus atomic registration (same block without await): connecting before, during or after startup always gives hello→STREAMING, without a duplicate race nor one with the first step. Single path (never queue/global broadcast); `server.sent` counts only what is drained (steps plus end).
- **steps, ephemeral:** broadcast on the fly; whoever enters midway receives the "now" (it is replay, you enter the now); Full drops counted, never resent.
- **end, best-effort delivery:** it carries the verdict; `put(timeout=2 s)` post-simulation (traci already closed: zero impact on dynamics) so it does not fall off the wire with the queue saturated at close; if even so it does not fit, it is counted.
- **`--ws-open-grace S`** (player): wait post-hello/pre-loop so viewers can connect; the whole run lasts seconds (~1400 streaming steps/s measured); pure wall time, passivity verified (md5 fixed in `tests/test_stream_protocol.py` case 7).
- **`--ws-stall-drain`**: delay per drained item (previously it also slept with an empty queue and accumulated a backlog of sleeps after the end).
- **`--ws-realtime`** (the only pacing mode, no free factor): the step with `t_sim = t` is emitted at the wall instant `t − t_sim0` from the first step (absolute anchor to t_sim; a lag does not accumulate). Demo and `--3d` always; certification without the flag. Passivity and anchoring verified (`test_realtime_pasividad_y_anclaje`; strict verification: 186.3 s in 186.3002 s, drift 0.2 ms).

## Limits

- `broadcast()` without backpressure means a client that does not consume receives until it fills its buffer and then websockets cuts it off (`ping_timeout`); the steps dropped by our queue are not resent: the network stream is best-effort, the truth is the log.
- Whoever connects after the end receives hello and the close → no verdict for that viewer (declared; the verdict lives in the log and in whoever received it).
- Tests: `tests/test_ws_emitter.py` (5): heading table, mapping, hello/end order plus t_emit absent, drop counted with complete log, real WS FIFO round-trip. Full WS suite: 19/19.

## References

- `docs/modules/ws_payload_contract.md`: the v3.0.0 contract that this emitter produces.
- `docs/modules/decided_tick_view.md`: the view that the emitter consumes.
- `docs/modules/ws_canonical.md`: the canonicalization (`canonical_bytes()`) of the payload.

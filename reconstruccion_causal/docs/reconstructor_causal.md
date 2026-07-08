# Causal reconstructor of the messages shadow (batch with guard)

> Permanent, self-contained document, mappable to the paper. It describes the
> FINAL STATE of the module.

Causal reconstruction of the second half of the messages shadow (V2I messages
→ shadow vehicles in SUMO). It corrects the three future-information leaks of
the previous base version (not included in this repository), which is kept
**intact as the comparison baseline**; its code is neither imported nor
referenced — the preserved formulas are physically copied with attribution in
comments).

## Causality criterion (the contract)

> To decide the state of a vehicle at instant `t`, only messages with
> `timestamp <= t + BUFFER` may be used, with `BUFFER = 1 s` (= emission
> delay). Depending on the complete trace invalidates the real-time
> reconstruction claim.

**Batch with guard** pattern: the full CSV is loaded into memory
(comparability over the Monte Carlo sweep), but consuming it beyond the
per-decision window is forbidden and verified through a **hard assert**
(`CausalityViolation` aborts the run). If the logic does not need the future,
the assert never fires — that PASS is the evidence that it would work live.

## Components

| File | Role |
|---|---|
| `scripts/guardia_causal.py` | `GuardedMessageStore`: the only access path to the messages. Clock-based delivery (`deliver_until`), per-decision views (`slot_view`, window `t+buffer`), transitive accounting (`check_state_use`), `assert`/`audit` modes. |
| `scripts/causal_core.py` | Math copied verbatim from the base (Veins transform, circular lerp, per-slot fill rule, snap) + `VehicleTracker` (incremental per-slot decision) + frame policy. |
| `scripts/causal_player.py` | Per-tick TraCI loop: deliveries → removals → creations → route extension → commands → step → validation. Standalone mode (headless certification) and attach mode (multi-client views). |
| `scripts/certificar_barrido.sh` | Certification matrix (3 frequencies × 4 levels, assert). |
| `scripts/comparar_vs_base.py` | Classified diff against the base; unexplainable difference = FAIL. |
| `scripts/lanzador_causal.sh` | Menu (adapted from the collaborator's launcher) for certification/views over the MC sweep. |
| `scripts/vista_dos_ventanas.sh` / `vista_comparada.sh` | Reality-vs-shadow views (two SUMO side by side / one overlaid SUMO). |
| `scripts/preparar_entrada.py` | Physical copy of the sweep CSV with a cosmetic ID shim (`v7→veh_7`). |
| `scripts/limpiar_heading_espurio.py` | Data-quality census tool (`--solo-censo`); its cleaning function was cancelled (the source defect was removed in the mobility). |

## Decision semantics (final state)

- **Clock**: SUMO time `t`. A message with `simTime = m` arrives at `m`
  (instantaneous emission; measured rx−sim latency ≤ 1 ms). The state shown at
  `t` represents the instant `t − delay`; with `buffer = delay = 1 s`,
  "messages ≤ represented_instant + buffer" ≡ "messages already arrived"
  (± `TICK/2` quantization).
- **Slot grid**: anchored at `t0` (the vehicle's first message), slot
  `k` → instant `tk = t0 + k·period`, applied at `tick_of(tk) + delay_ticks`.
  Identical to the base.
- **Per-slot fill** (base rule, preserved): message present →
  OBSERVED / REACQUIRED; closing anchor within `(tk, tk+buffer]` →
  INFERRED (interpolation between real ones); no anchor in the window → UNCERTAIN
  (constant-velocity dead-reckoning using only the past). Controlled snap when
  closing an UNCERTAIN zone with divergence > `reacq_snap_th` (3 m), flagged.
- **Frames (sub-slot, 10 Hz) — DR-until-usable-anchor policy**: consecutive
  slots are interpolated linearly; if the closing state embeds a message newer
  than `s_frame + buffer` (anchor still "not arrived"), the frame is
  **extrapolated by dead-reckoning with the past velocity** (the two last
  decided slots; `seg` suffix `(DR)`): the vehicle advances smoothly, and once
  the anchor enters the window it converges to the normal interpolation
  (measured adjustment: median 0.34–0.37 m). It never interpolates toward an
  anchor `> s + buffer`. The decision uses only the embedded timestamp
  (`max_msg_ts`) — the knowledge of the arrival clock — and the extrapolated
  content derives solely from the past (transitive accounting intact). The
  base's flagged snap (hold and jump) is preserved. With no previous slot (k=0)
  it degrades to hold: no velocity is invented.
- **Life cycle — extrapolation window and removal** (`--lost-after-s`,
  default 3 s; floor `period+buffer`): under silence, UNCERTAIN is extrapolated
  until the vehicle is declared **lost** once the window is exhausted; then it
  is removed. If messages resume, **re-acquisition**: the tracker persists
  (the `t0` grid and the arrived history are causal knowledge) and the vehicle
  is re-created in SUMO (staging ~delay, revealed at its exact tick). Removal by
  exhausted window is the ONLY termination mechanism and is causal precisely
  because it does NOT distinguish the cause of the silence (loss, leaving the
  map, shutdown): distinguishing them would require the future.

## The three corrected leaks (vs the collaborator's base)

1. **Removal (R2)** — the base removed at `last_tick+1` knowing the last
   message of the complete trace. Causal: extrapolation window → lost →
   removal (above).
2. **Route at creation (R1)** — the base created the vehicle with its complete
   path (norm_edges of the whole trace). Causal: it is born with a minimal
   route and is extended incrementally with the edges of the arrived messages,
   bridged with `findRoute` (a query to the static network: causal). `setRoute`
   requires the list to start at the current edge (official TraCI docs); the
   extension is deferred and retried. Vehicles that emit from the crossing's
   internal lanes (this happens for tens of seconds): the insertion edge is
   derived from the static network by following the outgoing connections of the
   internal lane (`resolve_exit_edge`) — infrastructure, not trace.
3. **Final silence tail** — the base did not extrapolate after the last message
   because the batch knew the end of the trace. Causal: the tail is
   extrapolated UNCERTAIN until the lost window is exhausted, like any silence.

**The guard as a verifiable guarantee**: every decision accesses messages via
`slot_view(veh, t)` (window `t+buffer`); the "does the next message fall in the
window?" peek lives inside the guard (equivalent to closing the window by clock
live); each state carries `max_msg_ts` and composing states re-verifies the
window (`check_state_use`) — "using a future anchor through an intermediate
state" is also caught. The summary reports
`guardia: {status, violations, accesses, max_lookahead_s}`.

## Verification (final state)

**Certification matrix** (assert, `interp linear`, DR policy +
`lost-after` 3 s, over the REGENERATED sweep): **12/12 PASS, 0 violations,
lookahead ≤ 1.000000 s** — under loss the lookahead saturates exactly at the
buffer and never crosses it.

| combo | guard | lookahead_s | states | frames DR | removals | reacq |
|---|---|---|---|---|---|---|
| 1Hz/P00 | PASS | 0.9480 | 21684 | 0 | 62 | 0 |
| 1Hz/P25 | PASS | 1.000000 | 20915 | 1485 | 66 | 4 |
| 1Hz/P40 | PASS | 1.000000 | 19963 | 1908 | 79 | 17 |
| 1Hz/P50 | PASS | 1.000000 | 19187 | 1962 | 89 | 27 |
| 2Hz/P00 | PASS | 0.4471 | 22000 | 0 | 62 | 0 |
| 2Hz/P25 | PASS | 1.000000 | 21790 | 268 | 62 | 0 |
| 2Hz/P40 | PASS | 1.000000 | 21466 | 568 | 63 | 1 |
| 2Hz/P50 | PASS | 1.000000 | 21007 | 748 | 65 | 3 |
| 5Hz/P00 | PASS | 0.1500 | 22201 | 0 | 62 | 0 |
| 5Hz/P25 | PASS | 1.000000 | 22067 | 2 | 62 | 0 |
| 5Hz/P40 | PASS | 1.000000 | 21855 | 22 | 63 | 1 |
| 5Hz/P50 | PASS | 1.000000 | 21507 | 36 | 63 | 1 |

**Negative controls** (a guard that never fires proves nothing):
- Unit tests `tests/test_guardia.py` (7/7): direct and transitive future access
  fire `CausalityViolation`; the window hides without firing; monotonic clock.
- Integration `--leak-probe retiro_batch` (reintroduces the base's R2 leak):
  fires at the first creation and aborts with exit 2.

**Equivalence with the base where there is no leak** (classified diff,
`comparar_vs_base.py`): common rows bit-identical; every difference falls into
classes explained by the corrected leaks or by the causal frame policy
(`FRAME_HELD_CAUSAL` = `(DR)` frames where the base interpolated with the
future); an unclassifiable difference = FAIL. Leak R1 is additionally verified
by `position_error_max = 0.0` (with `keepRoute=2` the route is bookkeeping and
does not alter the reproduction).

**Data**: Monte Carlo sweep regenerated with mobility
f712b746 + removal-at-edge (the driver cuts the samples after the end of the
exit edge and hardens the release: vehicles DISAPPEAR when leaving the frame;
driver md5 `00778f15…`, dataset `f712b746…` unchanged, messages GT verified
byte-identical). Census of the regenerated sweep: **0 spurious messages in
14 113 917 rows** (benign residual: ~1 row/file at 5 Hz with empty laneId and a
healthy heading — a 1-sample mapping micro-gap). 3003/3003 runs, 0 definitive
failures.

## Views and GUI stability

Two reality-vs-shadow setups (`vista_dos_ventanas.sh` preferred;
`vista_comparada.sh` overlaid with barrier synchronization). The shadow runs
with the guard in assert during the session. In GUI modes the staging uses
`--staging-mode park` (transparent parking inside the network, commanded only
once) and **clamping of every command to the network bbox**: sumo-gui 1.12
aborts non-deterministically (`assert(!ISNAN(lanePos))`, `lanePos`
uninitialized in `libsumo::Vehicle::moveToXY`) if a position outside the
network is commanded with `keepRoute=2`. In headless certification mode the
staging is the inherited one (`offnet`), stable and with certified outputs.
Additional defensive layer: degenerate angles are sent as the exact TraCI
sentinel (inert today: the regenerated sweep no longer carries corrupt angles).

## Passive observation layer — TraCI subscriptions

With `--dense-out` or `--occupancy-out`, each newly present shadow subscribes
ONCE to post-step read variables that arrive in the cache of the
`simulationStep` itself (zero additional TraCI round trips):

- `VAR_ROAD_ID (0x50)` / `VAR_LANE_ID (0x51)`: edge/lane mapped by SUMO
  (dense dump and occupancy rule).
- `VAR_SPEED (0x40)`: SUMO's post-step speed — the stopped-vehicle metric
  measures what the shadow REPRESENTS, not what the message said.
- `VAR_POSITION (0x42)`: post-step position
  for the WebSocket emitter's `pos_plane` (v3 contract, Section 5.1) of ALL
  revealed shadows, not only those applied on the tick. **Metric-plane
  verification** (three ways):
  official TraCI docs — 0x42 returns the Cartesian position `[m,m]` (the
  vehicle's front); the messages net has no geographic projection
  (`projParameter="!"`, `netOffset 0,0`), so geo is impossible here; and
  empirical test on this net: `moveToXY(57.0, 74.4)` → `getPosition` and
  `sub[VAR_POSITION]` return exactly `(57.0, 74.4)` (0.0000 m). It is the
  SAME plane used by all post-step validation.

Passivity: project criterion — `validation_log.csv` byte-identical with and
without the change (verified when extending the subscription: validation_log,
crossing_times and the occupancy series byte-identical; the same verification
with which the dense dump and the occupancy were certified).

## Declared limits

- **Negative information**: the guard watches ACCESSES to messages; that the
  UNCERTAIN classification is decided at the window's clock closure is a
  property of the code design, not of the assert.
- **Quantization**: `|delay_efectivo − delay| ≤ TICK/2` (same as the base).
- **Dead-reckoning**: assumes constant velocity; under long gaps it diverges
  from the real position — the honest cost of loss, bounded by the lost window
  (it does not extrapolate indefinitely).
- **Approximate insertion edge** for emitters from an internal lane.
- **Declared risk** (not observed: 0 setRoute failures): SUMO's
  `updateBestLanes` assert if the loss hides the next edge.

## SENSITIVITY DEBTS (pending for the paper — validity standards)

Every threshold is either justified or accompanied by a sensitivity analysis.
The following currently run with PROVISIONAL values without a sensitivity
sweep:

| Threshold | Provisional value | Where it lives | Pending |
|---|---|---|---|
| Hysteresis of the committed crossing (crossing detector, messages GT) | 3 m | shared protocol (`docs/modules/gt_mensajes_sim.md`, rule 3) | threshold sweep vs count FN/FP |
| Tolerance window of the GT↔shadow matching | 1.0 s (`matching_tolerance_s`, `config/sites/rusia/site.yaml`) | matching of entry events | sensitivity of artificial FN+FP vs tolerance |
| Extrapolation / lost window (`--lost-after-s`) | 3.0 s (floor period+buffer) | `causal_core.TrackerConfig` / CLI | threshold sweep vs teleports/flicker/DR divergence |
| Monte Carlo sweep repetitions | n = 100 per level×frequency | MC sweep script (offline, not included) | justify n by the CI width of the metrics (or an n sweep) |

Also provisional-inherited: `reacq_snap_th = 3 m` (the base's snap policy,
preserved) — if it enters results, the same rule.

## Usage

```bash
# prepare input (ID shim) from the Monte Carlo sweep
# (the sweep is generated offline and is NOT included in the repository)
python3 scripts/preparar_entrada.py \
    --src <ruta_al_barrido_mc>/2Hz/P30/rep000.csv \
    --dst inputs/cert/mc_2Hz_P30_000.csv

# certifiable causal run (assert; exit 2 if the guard fires)
python3 scripts/causal_player.py --csv inputs/cert/mc_2Hz_P30_000.csv \
    --net inputs/intersection_rusia.net.xml --out runs/<id> \
    --delay-s 1.0 --period-s 0.5 --interp linear --lost-after-s 3.0

# full certification matrix / classified comparison / views
./scripts/certificar_barrido.sh
python3 scripts/comparar_vs_base.py --base runs/<base> --causal runs/<causal>
./scripts/lanzador_causal.sh
```

Parameters: `--delay-s` (= buffer, > 0), `--period-s` (1.0/0.5/0.2),
`--interp {none,linear}`, `--lost-after-s` (3 s), `--guard-mode
{assert,audit}`, `--staging-mode {offnet,park}`, `--reacq-snap-th` (3 m),
`--leak-probe retiro_batch` (negative control).

Per-run outputs: `results/validation_log.csv` (base columns),
`results/summary.json` (+ `guardia`, `retiro_report`, `ruta_report`,
`lost_after_s`), `results/crossing_times.csv` (informational geometric logger;
the official crossing metric is the one from the shared protocol).

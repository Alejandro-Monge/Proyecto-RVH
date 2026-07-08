# Module — Causal-loop orchestrator (`digital_shadow/harness/`)

Assembles, frame by frame and without seeing the future, the shadow's causal loop: `source (tracks replay) → causal reconnection → streaming event commit → position/route → Sink (SUMO)`. The `Sink` decouples the drawing, so the causal core is measured without launching SUMO. Module: `orchestrator.py` and `sink.py`; integration audit in `scripts/audit_harness_part3.py`, live SUMO smoke in `scripts/smoke_harness_sumo.py`. It reuses the position, the route, the causal reconnection, the SUMO scenario, and the contract already built.

## Per-frame sequence

Per frame f at `t_v = f·frame_dt` (= `1/site.fps_ref`; 12/355 in rusia):

1. births → reconnection `on_birth`;
2. observations → chains (incremental stop-line crossing, running class vote, position filter from birth);
3. on detecting the entry crossing (plus ±3 window = 0.101 s) → commit origin and predicted destination → instantiate;
4. each frame → move;
5. deaths → `on_death`;
6. confirmed chain end (last death plus buffer) → observed destination → remove;
7. `sync(t_v)`.

Each commit seals its delay. The `Sink` decouples the drawing: `RecordingSink` (measurement, no SUMO, deterministic) records the orders; `SumoStageSink` translates them to live SUMO. This way the causal core is measured without launching SUMO.

## Integration audit

Four checks plus delay sealing:

- **A) Online stream matches the batch re-derivation:** the incremental online event commit gives **0 differences** (71/80 cross chains match, exit_arm/entry_arm/lane) against the batch re-derivation over the causal-reconnected tracks. Frame-by-frame processing introduces no difference.
- **B) Position filter warm at instantiation:** median **109 approach frames** processed before instantiation (min 5, **0 cold chains**); the filter arrives warm (fed from birth; for fragmented ones, from the fragment before the gap). The |v| at crossing is low because of the pos clamp near the end of the approach and because of starts from a standstill on green; it is not cold.
- **C) Deterministic, no dropping:** two runs give **30131 identical orders**. In measurement there is no dropping (the whole segment is replayed in order); the `FileVideoSource` realtime/drop mode belongs to visualization.
- **D) Pace:** **0.48 ms/frame** (orchestration only; video period 33.8 ms → ×70 margin). This figure covers only orchestration; the full loop with detector and SUMO is measured separately, and if it did not keep the margin it would be an additional delay to declare.

## Delay sealing (budget per commit type)

Distribution (not mean), s, shadow−GT_entry:

| commit | N | median | p10 | p90 |
|---|---|---|---|---|
| instantiation / origin / predicted destination | 64 | **0.37** | 0.03 | 0.74 |
| observed destination | 64 | **6.09** | 4.31 | 8.47 |

The predicted gives low delay (~0.37 s = shadow-GT offset ~0.23 s + the explicit 0.101 s line of the lane look-ahead + process); the observed pays the junction transit (~6 s). It is the delay-vs-accuracy curve turned into a budget. The 0.101 s line (±3 frames of the lane) enters explicitly, not as a footnote: it is a quantified and accounted-for causality violation.

## Integration findings

Assembly revealed two points; neither affects the causal core.

1. **SUMO auto-removes on route arrival** (cause confirmed with data: `dropped ⊆ arrived`, 21/48 in the smoke subsegment). The vehicle reaches the end of the egress ≈ leaves the intersection → reasonable removal, but it precedes the orchestrator's end-of-chain removal (death plus buffer). The sink handles it without affecting the loop. It remains to be reconciled in visualization (accept arrival as exit, or prevent it). A first hypothesis (far H error outside the network) was ruled out with data: the bounds-guard skipped 0 moves → the coords were inside the network; the cause is arrival, not H.
2. **The `move` uses `moveToXY(raw observed world)`**, which carries the homography error (the lateral-by-lane decomposition is applied at instantiation via the entry lane, not in the `move`). Refining the `move` to use the decomposition (moveTo lane/pos at egress via the predicted exit) is a visual-fidelity improvement for visualization; it does not affect the stream or the delays.

Neither affects the causal core (stream plus delays), validated separately with `RecordingSink` (checks A–D).

## Dependencies and assumptions

It reuses detector and tracker (replay of the tracks CSV = their online output), `CausalReconnector`, `VehiclePositioner`, `RouteCommitter`, `SumoStage`, and the causal contract. Fidelity is measured offline (it does not enter the harness). Reconnection uses the same order and decision as the previous stage (online, features given); the head-only-at-birth causalization is an additional strictness out of scope (see `causal_reconnect.md`). The position filter is fed only during the approach (its pos is only valid along the approach axis; after the crossing the pos comes from the observed `move`).

**Parameter `frame_dt`.** Constructor: `frame_dt: Fraction | None` (keyword). It is the medium's exact frame period, typically `1 / load_site_config(site).fps_ref`. It must be a `Fraction` (or `None`), never a float: the clock is computed `t_v = float(f * frame_dt)` with rational arithmetic and a single rounding; injecting a float introduces double rounding and changes 129 695 of 200 000 `t_v` in the last bit (first affected frame: f=3), which can flip a boundary `searchsorted` and move a decision tick. `None` = inherited default `Fraction(12, 355)` (rusia site), for the measurement instantiators that do not inject it (`run_harness_viz`, `run_harness_demo_long`, `audit_harness_part3`, `measure_harness`, `smoke_harness_sumo`, `evaluate_shadow_fidelity`, `validar_causal_completo`); their behavior is byte-identical. The demo's causal path injects `frame_dt=1/FPS_REF` from `run_harness_causal.py`.

## Semantics of the observed-destination commit

The **6.09 s [4.31, 8.47]** in the table corresponds to the production core as it stands: the observed-destination commit is emitted at chain end plus buffer, and it pays the full junction transit plus the reconnection wait.

The permanent causal command (`scripts/run_harness_demo_causal.py`, subclass `ChainCausalExit`) redefines the commit instant: it is emitted at the exit stop-line crossing, detected incrementally (information available live at that moment, ±3 frame window declared). Measured against the same GT: **median 0.39 s [p10 0.20, p90 0.76], n=64**; process latency (commit − own crossing) 0.152 s. The chains and the lane/presence/delay metrics vs GT are identical (validated field by field); what changes is which physical event the commit is sealed against. When citing either of the two numbers, its semantics must be declared; the delay-vs-accuracy curve ("observed pays 6–8 s") refers to the old semantics and should be re-derived if the core adopts the causal one.

## References

- `docs/modules/causal_reconnect.md`: the causal reconnection that feeds the loop.
- `docs/modules/position.md` and `docs/modules/route.md`: position and route that the orchestrator commands.
- `docs/modules/sumo_stage.md`: the SUMO stage that `SumoStageSink` translates to.
- `docs/modules/causal_contract.md`: the causal contract that the assembly respects.

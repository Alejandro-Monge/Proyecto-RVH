# Module — SUMO stage (`SumoStage`)

`SumoStage` wraps TraCI to use SUMO as a render surface where the shadow **dictates** the
position of each vehicle (position-driven), not as a simulator that drives. It provides the life cycle
`add` / `place_*` / `remove`, a slave clock and the junction gap filling. The code lives in
`digital_shadow/sumo/stage.py`; the figures are verified against SUMO 1.12.0 in execution.

## Requirements

Verified with SUMO running:

- **SUMO control disabled** per vehicle: after `add`, `setLaneChangeMode(vid,0)` +
  `setSpeedMode(vid,0)`. Without this, SUMO changes lane / car-following and deflects the vehicle ~1
  width (measured 3.2 m); with control disabled, `moveTo` is faithful (0.026 m). Mandatory.
- **Slave clock, absolute re-pointing:** `sync(t)` = `simulationStep(float(t))` brings the SUMO
  clock to the video time t. Verified: **|err| max 0.49 ms/frame, 0 ms accumulated drift**
  (step-length 0.001). It resolves the 30 Hz vs 29.583 mismatch (rates are not equalized; it re-points
  to the rational time `f·12/355`).
- **Gap filling along the internal lane** (not delegated to SUMO, which with control disabled does not
  advance): `internal_shape(entry_arm, exit_arm)` requests the polyline from `traci.lane.getShape`
  (id of `lane_map.junction_internal`, for example `:J5_6_0` for south→west) and `GapFiller` interpolates
  the position at the last observed velocity. Verified: the filling follows the **curve** (deviation
  8.26 m from the straight line, approximately the corner cut it avoids), it does not teleport it.

## API

- `start()` / `close()` (or `with`): launches `sumo` (headless, measurement) or `sumo-gui` (visualization).
- `add(vid, edge, lane_index, pos)`: instantiates with control disabled.
- `place_lane(vid, lane_id, pos)`: `moveTo` (approach/egress; position piece).
- `place_xy(vid, x, y, keep_route=2)`: `moveToXY` (junction/filling). **`keep_route=2`** (exact xy without
  snap) — verified: places at 0.000 m; `keep_route=1` also works if the point is on the route, but 2
  is robust for any point, so **2** is used.
- `set_target(vid, exit_edge)`: `changeTarget` (logical route / O-D; the position we set ourselves).
- `internal_shape(entry_arm, exit_arm)`: polyline of the internal lane (from SUMO, not from the net file).
- `sync(t)`: re-points the clock and returns `getTime`. `remove(vid)`, `position(vid)`, `lane_of(vid)`.

**Pure interpolation** (without SUMO, testable): `polyline_arclen`, `point_at_arclen` (clip to [0,length]),
`GapFiller(shape, speed, t0).at(t)` (advances `speed·(t−t0)` over the polyline — backward-only state).

## Verifications

Four checks with SUMO running (`scripts/smoke_sumo_stage.py`):

1. **clock sync:** |err| max 0.49 ms, no drift.
2. **keepRoute junction:** keepRoute=2 → 0.000 m over `:J5_6_0`.
3. **fill by curve:** deviation 8.26 m from the straight line (follows the curve).
4. **life cycle:** add → approach → junction → egress → remove, without error.

## Configuration

- It reads the internal lane from `config/sites/rusia/sumo/lane_map.yaml: junction_internal` (derived from the net,
  additive, `derive_sumo_lane_map.py`, control 7 = 12/12 movements). The **geometry** (shape) it requests
  from SUMO via TraCI, not from the net file, so the runtime does not read the net.
- It imports TraCI lazily (importing the module does not require SUMO).

## Limits

`SumoStage` is only the position-driven stage. Left out, in other modules:

- **Causal perception** (online reconnection; crossing/event and lane/class commit).
- **Orchestrator** of the causal loop (life cycle from the commits and delay sealing).
- **Visualization modes** (sumo-gui + video + delay HUD) and **measurement modes**.

## References

- `docs/modules/sumo_lane_map.md`: the approach/lane↔edge/lane mapping and `junction_internal`.
- `docs/modules/position.md`: the position piece that uses `place_lane`.
- `docs/modules/harness_orchestrator.md`: the causal orchestrator that uses this stage.
- Site config: `config/sites/rusia/sumo/lane_map.yaml`.
- `scripts/smoke_sumo_stage.py`, `tests/test_stage.py`.

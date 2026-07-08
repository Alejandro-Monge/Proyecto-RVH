# Module — Stream entities (`ws_emitter/web/js/entities.js`, `entities_core.js`)

3D client of the messages profile that draws the stream vehicles. The pure core
(`entities_core.js`) is verified in an automated way; the three.js render is validated
visually.

## Separation of responsibilities

- **`entities_core.js`** (no THREE, no DOM): `reconcile(prevIndex, step, layers,
  vclassDims)` → `{create, move, recreate, remove, nextIndex, ignored, unknownVclass}`.
  Rules: `source ∉ layers` → count and list, do not draw; `vclass ∉ vclass_dims` → count,
  do not draw (no invented dims); different `generation` → `recreate` (another incarnation,
  without tween: it is not interpolated). Invariant: `nextIndex` contains exactly the
  drawable ids of the step.
- **`entities.js`** (junction layer with THREE): `EntityLayer(scene, hello).apply(step)`
  applies the actions to the meshes. It reuses `sumoToThree`/`headingToRotationY` from
  `net_scene.js`: vehicles and roadway share the same mapping, and a second copy
  would diverge.

## Heading coherence

With the mapping `y → −z`, rotating the local `+x` by θ over `+Y` gives
`(cosθ, 0, −sinθ) = east·cosθ + north·sinθ`, exactly the contract direction
(rad, 0 = +x east, CCW). The payload heading is applied as is as `rotation.y`, without
ad hoc signs. The chirality of the mapping is validated statically (net_scene); this is its
dynamic verification, confirmed with real data.

## 3D model of the vehicle

- The `sumo` source is drawn as a `THREE.Group` of several boxes (low-poly model:
  car = body + cabin; bus = long tall box; truck = cabin + cargo), not a single
  box. The shapes live in `entities_geometry.js`, a pure module (no THREE) so that the
  settling is verifiable in node. Scaled to `vclass_dims`; presentation heights
  (`HEIGHT_M`).
- **Settling:** the base of the model (wheels) rests on the visible roadway surface
  `ROAD_SURFACE_Y = 0.02` (= `Y_OFF.lane` of net_scene), not on Y=0 (which would end up under the
  junction). Mesh origin at the base (y=0 local), front at x=0 (`pos_plane` =
  bumper). Measured in `tests/vehicle_settle_check.mjs` (three in node: bounding box
  → `base.y == 0.02` in car/bus/truck; front at `pos_plane`). If `Y_OFF.lane` changes in
  net_scene, `ROAD_SURFACE_Y` changes.
- **Fidelity by dimensionality:** the vehicle (`sumo`) is a raised model (precise);
  the queue (`vision_box`) and the handoff are still a flat footprint (uncertain). The distinction
  is not of color but of volume.

## Geometry and styles

Declared presentation constants, not data:

- Flat footprint (`vision_box`/`handoff`): `BoxGeometry` centered, base on the ground. The
  `sumo` vehicle has its origin at the front bumper: `pos_plane` is the front
  (VAR_POSITION 0x42; same convention as the waiting boxes).
- Heights (car 1.5 / bus 3.2 / truck 2.6 m) and base colors (echo of the player's SHADOW_DIMS
  palette).
- `provenance` → style (legend on screen): OBSERVED solid saturated; INTERPOLATED
  normal; INFERRED bluish tint; UNCERTAIN translucent (0.45); HELD dimmed gray;
  REACQUIRED/REACQUIRED_SNAP magenta mark. It is the paper's uncertainty narrative
  made visible.
- Shared materials and geometries → removal is `scene.remove` (no dispose of
  shared ones; no pool: at most a few dozen vehicles).

## Pacing

The shadow replays at the speed at which it happened: realtime or nothing, no free factor (in
vision the video playback already fixes it; messages respects the same rule). Single mode,
`--ws-realtime`. Anchored to `t_sim` (the intersection clock, the contract's, not the
tick grid nor `--period-s`): the step with `t_sim = t` is emitted at the wall instant
`t − t_sim0` from the first step; the targets are absolute against the anchor, so a
lag does not accumulate debt. In demo and `--3d` it is always applied; in certification and measurement the
flag is omitted (raw, byte-compare intact).

The correct anchor is `t_sim`, not the simulation tick grid (`(i−i_begin+1)·TICK/F`):
sleeping by scaling the grid measures 1.00× sim/wall in standalone (they coincide numerically
there), but it introduces a free factor that violates the rule.

## Verifications

- **Passivity:** `test_realtime_pasividad_y_anclaje` compares the raw `validation_log` with
  the realtime one over the same artifact, byte for byte; drift ≤ 2 ms; the raw one does not sleep.
- **Anchoring (strict verification):** a complete run replays 186.3 s of `t_sim` in
  186.3002 s of wall (`realtime_drift_s = 0.0002` ≤ 0.002, the stage quantization; in
  `summary.ws_emitter`). The forbidden alternative is client-side interpolation.
- `tests/entities_core_check.mjs` (node v22, stream_harness pattern): create/move/remove;
  `generation` → recreate; layers count and list; unknown vclass; the invariant with no
  loose entities or leaks under 500 pseudorandom steps (deterministic PRNG) with
  simulated bookkeeping of the junction layer (`meshes == nextIndex` at each iteration).
- Import graph (auto-glob picks up `dev_entities.html`) and `node --check`. Full WS
  suite: 63/63.

## Visual validation

Command:
```
A: python3 -m http.server 8300 --directory ws_emitter/web
B raw:   cd reconstruccion_causal && ../.venv/bin/python scripts/causal_player.py \
             --csv inputs/cert/mc_2Hz_P00.csv --net inputs/intersection_rusia.net.xml \
             --out /tmp/ws3d --period-s 0.5 --occupancy-out /tmp/ws3d/occ.csv \
             --ws-log /tmp/ws3d/ws.jsonl --ws-port 8931 --ws-open-grace 15
B REALTIME (the demo mode): ... + --ws-realtime        (real speed, 186 s)
Browser: http://localhost:8300/dev_entities.html
```

Criteria the render must meet:

| # | Criterion | Verifies |
|---|---|---|
| 1 | Vehicles in the correct approaches, moving in the correct direction (pos+heading over real data) | DYNAMIC chirality of the heading over real data (the counterpart to the static one in net_scene) |
| 2 | Classes distinguishable by dimension (bus 12 / truck 7.5 / car 5 m) | scale to `vclass_dims` |
| 3 | Provenance visible | style by provenance |
| 4 | Movement at REAL SPEED with `--ws-realtime` | anchoring to `t_sim` (186.3 s t_sim = 186.3002 s wall, drift 0.2 ms) |

## Aesthetic presentation

On top of the already-validated fidelity base, without hiding it, the aesthetic presentation adds:

- Interior of the crossing as a uniform surface (junction with no loose connectors).
- Lane lines and stop line from config, not hardcoded.
- 3D car/bus/truck models instead of `BoxGeometry`, scaled to `hello.vclass_dims`:
  procedural, not `.glb` (deterministic, pinnable, no sourcing).

Video-style lighting is not implemented: the video is flat and overcast, and `MeshBasic` already
approximates it.

## References

- `ws_net_scene.md`: the shared mapping `sumoToThree`/`headingToRotationY` and the static
  roadway.
- `ws_payload_contract.md`: the payload contract (layers, provenance, `vclass_dims`).
- `ws_stream.md`: the WS consumer that delivers the steps.

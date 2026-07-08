# WebSocket payload contract v3 — 3D web visualization of the digital shadow

The contract is stable at **`schema_version = 3.0.0`**; any later change requires a new semver version. Its scope is the presentation demo (PEWASUN 2026), not a result. The emitter interface is `decided_tick_view.md` (anti-leak layer 1): the emitter consumes that view, never raw state.

## Principles

1. **Pure renderer client.** The Three.js client draws what arrives; it computes nothing derived (no projections, no counts, no semantic interpolation). Everything derived is resolved server-side at the post-`simulationStep` point.
2. **Two mandatory representations.** (a) The box→vehicle handoff gap in vision is represented, not disguised; (b) the stream is a replay of precomputed artifacts and it declares so (`mode`), never presented as a live pipeline.
3. **Two sides measurable separately.** server→WS is measured with `t_sim`/`seq`; WS→render with `t_emit` (server monotonic clock) against the client clock.
4. **Nothing of the concrete video embedded.** Rate (`dt`), frame geometry and site travel in the session message from config; the client assumes none.
5. **No claim in advance.** The causality verdict of the run does not exist until the end: per step there are counters; the verdict goes in `end`.

## Topology and profiles

One WebSocket stream per shadow. Single schema, two profiles:

- `shadow = "messages"`: one source (`sumo`). Reconstructor: `causal_player.py` (reconstruccion_causal), post-step hook of its per-tick loop.
- `shadow = "vision"`: two sources the client composes: `sumo` (post-stop-line vehicles) + `vision_box` (queue of the vision layer, which does not live in SUMO by the causal rule) + `handoff` (transition between the two). Reconstructor: `run_harness_causal.py` (demo_causal), post-step hook of `run_live`.

Wire format: JSON, one message per logical frame. The transport details (reconnection, backpressure, binary) are left for the construction phase and verified against the official doc; the contract fixes the logical schema. `seq` is monotonic within a session; the semantics after reconnection are outside v3.

## Message types

Three types: `hello` (1, on open), `step` (N), `end` (1, on close). The client must reject a `step` without a prior `hello` and treat the absence of `end` as a run without a verdict (not as PASS).

### hello — session constants

| Field | Type | Semantics | Source (code) |
|---|---|---|---|
| `type` | `"hello"` | discriminator | — |
| `schema_version` | string semver | version of this contract (`"3.0.0"`) | contract constant |
| `shadow` | `"vision"\|"messages"` | stream profile | server instance |
| `mode` | `"replay"\|"live"` | nature of the source. Only `replay` implemented; `live` reserved for the migration to camera | replay evidenced: vision precomputes the whole reconstruction before the loop (`build_veh_causal`, `run_harness_causal.py`); messages loads the complete CSV after the guard (`load_rows` + `GuardedMessageStore`, `causal_player.py`) |
| `dt` | float (s) | period of the logical frame/tick. Constant per run; the client does not assume a rate: it reads it from here | vision: `DT = float(1/site.fps_ref)` (`run_harness_causal.py`); messages: `core.TICK` (`reconstruccion_causal/scripts/causal_core.py:33`, single declared source) |
| `site` | string | site id | `config/sites/<site>/site.yaml` (`site_id`) |
| `frame_geometry` | `{width:int, height:int, source:"site.yaml"}` | geometry of the video frame, **necessary to interpret `bbox_px`**; without it the concrete-video assumption is reintroduced | `SiteConfig.frame_width/frame_height` (`digital_shadow/config/site.py`); validated against the zones in `zones.py::load_approach_zones` |
| `net_md5` | hex string | pin of the SUMO network; the client validates that its static scene was derived from the same network | vision: md5 of the net in `common.check_provenance(INPUTS)` (`run_harness_causal.py::INPUTS`); messages: md5 of `args.net` (the emitter computes it over the file) |
| `artifact_md5` | `{name: hex}` (plural) | pins of the reproduced artifacts; they declare which replay it is | vision: md5 of tracks CSV + events CSV (`DATA[which]`, the emitter computes it); messages: md5 of `args.csv` + `lane_map_md5` (already computed, `causal_player.py` OCC block) |
| `occ_cfg_md5` | `{name: hex}` | pins of the occupancy config | vision: `approach_zones.yaml`; messages: `occupancy.yaml` + lane_map (idem) |
| `guard_mode` | string | regime of the causal guard | messages: `args.guard_mode` (`assert\|audit`); vision: `"assert_end"` (the probe is assert at close, `SondaTotal.verify`) |
| `zone_coverage` | `{brazo: {carril: {coverage, note}}}` \| null | declared coverage of the lane zones (`full\|partial`); the HUD does not present a partial count as a total | `LaneZone.coverage/coverage_note` (`digital_shadow/occupancy/zones.py`); `null` in messages (the SUMO lanes cover the full approach by construction, `occupancy_core.py`) |
| `layers` | list of strings | active sources of the stream. messages: `["sumo"]`. vision: `["sumo","vision_box","handoff"]`; **`vision_box` is mandatory in `shadow="vision"`** (the queue is constitutive of the vision shadow, see The queue as a source): a vision hello without `vision_box` declares a stream **not conformant to v3** (degraded debug mode), not an alternate profile. The client rejects entities whose `source` is not declared here, and for a non-conformant vision stream it labels it as degraded | vision: `args.waiting_boxes` of the entry point (`run_harness_demo_causal.py:54`, `BooleanOptionalAction default=True`) |
| `vclass_dims` | `{vclass: {length_m, width_m}}` | dimensions per class so the client draws without inventing | vision: `vehicletype.getLength/getWidth` of the real vTypes (`WaitingBoxLayer.dims`); messages: `SHADOW_DIMS` (`causal_player.py`) |
| `occupancy_semantics` | object | declaration of the profile's `occupancy` block (see occupancy): `kind`, halting threshold if applicable | messages: `halting_speed_mps` from `occupancy.yaml`; vision: label `conteo_crudo` (docstring of `zones.py::occupancy_counts`) |

### step — post-step state of a tick

| Field | Type | Semantics | Source |
|---|---|---|---|
| `type` | `"step"` | discriminator | — |
| `seq` | int ≥ 0 | strictly monotonic in the session | loop index (`fi` vision / `i` messages) |
| `t_sim` | float (s) | **single semantics across shadows: time reported by SUMO after the step (`getTime()`), never the imposed logical clock.** The logical clock is derivable as `seq · hello.dt`; the divergence `\|t_sim − seq·dt\|` is bounded by the quantization of the stage step-length (no accumulated drift: smoke `smoke_sumo_stage.py` with threshold < 2 ms and verification of 0 ms over 5325 frames, `sumo/stage.py` docstring) and is the server→WS side measurement, comparable across shadows because both emit what is reported | messages: `traci.simulation.getTime()` post-step (`sim_after`, already read); vision: `getTime()` post-step, a new read by the emitter (today `run_live` does not do it; `SumoStage.sync` already returns it, a minor change required) |
| `t_emit` | float (s) | server monotonic clock at serialization (WS→render side measurement). Not comparable across sessions | emitter runtime (construction); excluded from the canonical hash of the prefix audit |
| `entities` | array | see entities | `DecidedTickView` |
| `occupancy` | object | see occupancy | idem |
| `guard` | object | accumulated causality counters up to this step, never a verdict | messages: `store.guard_report()` (accesses, violations, max_lookahead, live counters); vision: emitter counters + `sonda.n_extrap`/`max_interp_gap` (`SondaTotal`) |

### entities[] — one entry per visible entity

| Field | Type | Semantics | Source |
|---|---|---|---|
| `id` | string | **canonical id per shadow**: opaque to the client and stable across `source` (the same physical identity keeps its `id` from box→handoff→vehicle). Vision: chain root (`vid = f"v{root}"`, `orchestrator.py:66`; box = same root). Messages: vehicle id from the CSV | the view canonicalizes it |
| `source` | `"sumo"\|"vision_box"\|"handoff"` | source of the entity. **`vision_box` and `handoff` only exist in `shadow="vision"`**; in messages everything is `sumo` | view |
| `provenance` | enum (see provenance) | quality/origin of this tick's state | messages: `st["provenance"]` of the applied state + HELD policy; vision: branch of `causal_pos` / box coasting |
| `pos_plane` | `[x,y]` (m, SUMO plane) \| null | post-step position. **null only in `source="handoff"`** (during the gap there is no position in any source; emitting it would be inventing it) | `sumo`: post-step TraCI read (pattern demonstrated in `run_readout`; in messages it requires extending the subscription with position, a declared minor change); `vision_box`: anchored position decided by the layer (`WaitingBoxLayer.draw`) |
| `last_pos` | `{x,y,t}` \| absent | only if `pos_plane=null`: last decided position of the box, with its instant. The client draws the ghost; the datum is labeled as past | handoff registry of the view (captured when the box goes extinct) |
| `heading` | float \| null | orientation in the contract convention (see Heading convention); the emitter converts at the boundary. null if there is no source | `sumo`: post-step TraCI angle (vision: mapped by SUMO after `moveToXY keepRoute=1`, declared as a presentation heading); `vision_box`: lane angle (`lane_angle_towards_stop`, mathematical radians, converted); `handoff`: null |
| `vclass` | `"car"\|"bus"\|"truck"` | closed vocabulary | messages: inverse of `VTYPE_MAP`; vision sumo: `getTypeID` post-step → vTypes map; box: class vote (`WaitingBoxTracker`) via `COCO_VTYPE` |
| `generation` | int, default 0 | incarnation number (re-acquisition after removal). The client does not interpolate across a change of `generation` | messages: `generation` dict (`causal_player.py`); vision: always 0 |
| `bbox_px` | `[x1,y1,x2,y2]` \| null | raw box in pixels of the present frame (interpretable only with `hello.frame_geometry`). **Only `vision_box`**; optional (HUD/debug) | columns `x1..y2` of the tracks CSV, row of the current frame |
| `carril` | int \| null | **Only `vision_box`**: voted lane of the box (causal vote from the track's past). null = background with no lane zone | return of `WaitingBoxTracker.update` (`lane_votes` vote) |
| `anchor` | `"lane"\|"raw_homography"` | **Only `vision_box`**: quality of the spatial anchoring of this entity (see The queue as a source). `lane` = lateral fixed to the center of the voted lane + filtered homographic longitudinal (position.py doctrine); `raw_homography` = sin_carril fallback: raw homographic position, with the known lateral error of the homography (~47 % of its energy). **The client must draw `raw_homography` visually distinct**: it is the visual validation mechanism, not a style detail | branches of `WaitingBoxLayer.draw` (`run_harness_causal.py`): `carril is not None` → anchored; else → raw |

**Closed key set for `source="handoff"`:** `{id, source, last_pos, vclass}` (+`provenance` fixed to `HELD`). **Forbidden** every prospective field (ETA, duration, progress): in a live pipeline it would not be known; the replay cannot allow itself to know it. The handoff entity is born from the observed event "the box went extinct" and dies from the observed event "the vehicle appeared" (`decided_tick_view.md`, Handoff registry).

**Ghost exclusion (messages):** only revealed shadows are emitted, the same eligibility rule as the occupancy (`activo ∧ no parked ∧ last_applied≠None`, `causal_player.py` OCC block). Vehicles in staging/park are internal SUMO presentation and do not appear in `entities`.

### occupancy — discriminated union (not uniform across shadows)

```
occupancy = { kind: "sumo_lanes",  per_lane: {brazo: {carril: {ocupacion:int, detenidos:int}}} }
           | { kind: "pixel_zones", per_arm: {brazo: {total:int, lanes:{k:int}, sin_carril:int}} }
```

- `sumo_lanes` (messages): computed in-loop post-step by `occupancy_core.occupancy_counts` (`causal_player.py` OCC block): speed = VAR_SPEED post-step, strict halting `< halting_speed_mps` (in `hello.occupancy_semantics`), explicit zeros. It is the same series as the paper metric.
- `pixel_zones` (vision): `zones.occupancy_counts` over the ground contacts of the present frame (`digital_shadow/occupancy/zones.py`). It is the **raw instantaneous count** (module docstring): no θ dwell, no identity; **it is not the paper metric** and the HUD must label it so (label in `hello.occupancy_semantics`). `detenidos` does not exist in this profile (there is no speed semantics); `sin_carril` does not exist in `sumo_lanes`.

### provenance — superset enum and valid subsets

Contract superset:
`OBSERVED | INTERPOLATED | EXTRAPOLATED | INFERRED | UNCERTAIN | REACQUIRED | REACQUIRED_SNAP | HELD`

| Shadow / source | Valid subset | Mapping from the code |
|---|---|---|
| messages / `sumo` | OBSERVED, INTERPOLATED, INFERRED, UNCERTAIN, REACQUIRED, REACQUIRED_SNAP, HELD | native labels of the applied state (visible in the summary, `causal_player.py`); HELD when the tick re-commands `last_applied` without new state or the interpolated frame is frozen (`held_frame`) |
| vision / `sumo` | OBSERVED, INTERPOLATED, EXTRAPOLATED, HELD | branches of `causal_pos` (`run_harness_causal.py`): observed sample → OBSERVED; interpolation ≤2·DT → INTERPOLATED; causal extrapolation over the path (`n_extrap`) → EXTRAPOLATED; frozen without path → HELD |
| vision / `vision_box` | OBSERVED, HELD | box present in the frame → OBSERVED; coasting in a gap ≤ `merge_gap_s` (visible without observation this frame) → HELD |
| vision / `handoff` | HELD (fixed) | by definition: only frozen past |

Note: `INFERRED`/`REACQUIRED*` are native to the messages reconstructor and are transported as is; that is why the contract superset is broader than the 5-value enum that suffices for the audit.

### Heading convention

**Radians, 0 = +x of the SUMO plane (east), counterclockwise, range [0, 2π).** The mathematical convention of the coordinate plane itself is chosen (the same as `lane_angle_towards_stop`). The emitter converts every source to this convention at the boundary. The exact semantics of the angle TraCI returns is verified against the official SUMO/TraCI documentation; the conversion lives at a single point of the emitter and is adjusted with that verification, without touching the contract.

### The queue as a source: status and uncertainty propagation (normative)

The waiting queue is **constitutive of the vision shadow**, not an optionally-enableable layer. `shadow="vision"` without a queue is not a valid profile: it is an incomplete shadow. The contract declares it with three normative clauses:

1. **The spatial queue is a presentation layer with declared uncertainty, not a validated metric.** Its status in the origin system: a layer active by default at the demo entry point; the paper occupancy metric is the series of the `occupancy` block / the measurement layer, never the box position. The client and any derived material (demo video, capture for the paper) must label the queue as presentation. This clause exists so the contract states it explicitly and the client does not have to assume it.
2. **Uncertainty travels per entity, not by footnote:** the `anchor` field (see entities) marks each box as anchored to the lane (`lane`) or as a raw-homography fallback (`raw_homography`, the sin_carril background of the south, known lateral error ~47 % of the homographic energy). The client draws the two states visually distinct; thus the visual validation is done over the 3D, with the uncertainty in view, without hiding.
3. **`vision_box` is mandatory in the vision stream.** `hello.layers` (see hello) declares it; its absence does not configure a legitimate alternate profile: the stream is **not conformant to v3** and the client labels it as degraded. `--no-waiting-boxes` can keep existing as a harness debug mode, but the contract declares explicitly that the stream it produces is not a conformant vision stream.

Real state of the layer in the code: the flag that governs the execution of the user command is the one at the entry point (`run_harness_demo_causal.py:54`, `BooleanOptionalAction default=True`, ON, with the exception of `--readout`, which never draws boxes so as not to contaminate the measurement). The `store_true` of the internal argparse of `run_harness_causal.py` only applies to the direct invocation of the internal module; its help and the docstring of `WaitingBoxLayer` are aligned with this behavior.

### end — stream close

The verdict lives here and only here.

| Field | Type | Semantics | Source |
|---|---|---|---|
| `type` | `"end"` | discriminator | — |
| `seq_final` | int | last `seq` emitted | emitter |
| `sonda` | `{status:"PASS"\|"FAIL", detail}` | final causality verdict of the run | vision: `SondaTotal.verify()` (+ emitter probe, layer 2); messages: final `store.guard_report()` (status/violations) |
| `guard_final` | object | final guard counters | idem |
| `totals` | `{steps, entities_max, handoffs}` | session summary | emitter |

A stream that ends without `end` has no verdict: the client shows it as "unverified run", never as an implicit PASS.

## Minimum profile per shadow (normative summary)

- **messages:** `entities[].source` ∈ {sumo}; `occupancy.kind = sumo_lanes`; `generation` active; no `bbox_px`, no `last_pos`, no `zone_coverage`.
- **vision:** `source` ∈ {sumo, vision_box, handoff}; `occupancy.kind = pixel_zones`; `generation` = 0; `bbox_px`/`carril`/`anchor` in boxes. The queue is constitutive: `vision_box` mandatory in `hello.layers`, with the status and uncertainty propagation of The queue as a source. A vision stream without `vision_box` (`--no-waiting-boxes`, debug mode) is not conformant to v3 and the client labels it as degraded; there is no "vision profile without a queue".

## Minor changes required in the implementation

No field of v3 lacks an honest source. Three require a declared minor change:

1. **`pos_plane` post-step for all messages vehicles:** the current subscription brings `VAR_ROAD/VAR_LANE/VAR_SPEED` (`causal_player.py`); the position variable must be added to the subscription (or read position per vehicle). Without this only the applied-this-tick ones have a post-step read.
2. **Post-step read in `run_live` (vision):** the current loop does not read back after the step; the pattern exists, demonstrated in `run_readout`. The emitter adds that read via the `DecidedTickView`, including `getTime()` for `t_sim` (single step semantics; `SumoStage.sync` already returns it, `sumo/stage.py`).
3. **`t_emit`:** it is born from the emitter runtime.

## Open decisions for the client

Marked, not resolved in v3:

1. **Handoff expiration.** The view registry expires ghosts at the R2 timeout (`lost_track_buffer_frames·dt` ≈ several seconds, `site.yaml`) as a defensive safeguard, but the real box→vehicle gap is ~0.37 s median (measured instantiation lag). A ghost that lasts R2 does not represent that gap. How the client presents it (visual duration, fade, cut) and whether the emitter needs a presentation expiration shorter than the defensive one is decided when building the client, with the measured gap in view, not before.
2. **`heading` of SUMO vehicles in vision is cosmetic, not a measurement:** it comes from the SUMO mapping after `moveToXY(keepRoute=1)` (see entities), not from the reconstructor. The client uses it only to orient the 3D model and does not present it as orientation fidelity; no derived material cites it as a result.

## References

- `docs/modules/decided_tick_view.md`: the emitter interface (anti-leak layer 1) that originates this payload.
- `docs/modules/ws_emitter_impl.md`: the message emitter that produces the payload conformant to this contract.

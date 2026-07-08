# Module — Vision shadow emitter (`from_vision_tick` of the `DecidedTickView`)

Second shadow of the 3D web. It builds the `DecidedTickView` of the vision profile by composing
three sources (versus the single one of messages): `sumo` (post-stop-line, causal reconstruction),
`vision_box` (spatial queue) and `handoff` (observed bridge). Emitter and v3 contract are
shared with messages (canonical, layers, provenance superset); `from_messages_tick` stays
intact. It lives in `scripts/demo_causal/run_harness_causal.py` (loop `run_live`, causal criterion
`place_causal`, reconstruction `causal_pos`, queue `WaitingBoxLayer`) and the view in
`ws_emitter/view.py`.

## Post-step readback by subscription

After `tr.simulationStep(t_v)`, `run_live` reads `tr.vehicle.getAllSubscriptionResults()`: the
position, angle, lane and edge of what was revealed (what SUMO actually drew, that is, only what was
commanded; causal by construction). It feeds `sumo_entities` in the assembly.

- Subscription, not `getPosition` in a loop (messages doctrine). `_sub_drawn(tr, vid)`
  subscribes to `_DRAWN_VARS = [VAR_POSITION, VAR_ANGLE, VAR_LANE_ID, VAR_ROAD_ID]` when
  `live[vid]=True` returns and after each re-add (the R4 remove+add rule loses the subscription, so
  it re-subscribes).
- TraCI constants: literal values `0x42/0x43/0x51/0x50` (SUMO_HOME is not set when
  importing the module), checked against `traci.constants` in `_setup` at runtime (assert).
  Verified in the environment: `VAR_POSITION=0x42, VAR_ANGLE=0x43, VAR_LANE_ID=0x51,
  VAR_ROAD_ID=0x50`.
- **Passivity:** read-only, it does not command, so it does not alter the dynamics (verified by
  byte-compare, see Passivity and provenance verifications).
- **Fidelity:** the subscription returns exactly the same as the direct getters,
  tested on the real readback path: 4010 comparisons, 0 discrepancies
  (`getPosition/getAngle/getLaneID/getRoadID` vs subs).

## Provenance in `causal_pos`

`causal_pos` returns `(pos, prov)`; the position is byte for byte the same, only the
label of the branch taken is added. Mapping (v3 contract, Section 3.5, subset vision/`sumo`):

| Branch of `causal_pos` | Condition | provenance |
|---|---|---|
| on a reconstruction node | interp with `w ≤ 1e-9` (tshift over `rt[i]`) | `OBSERVED` |
| between nodes | gap `ts[i+1]−ts[i] ≤ GAP_DT` (=2·DT) | `INTERPOLATED` |
| causal extrapolation over the path | gap > GAP_DT or post-end, `seg` present (`n_extrap`) | `EXTRAPOLATED` |
| frozen without path | `seg is None` (raw) | `HELD` |
| no position | `i < 0` | `(None, None)` |

`OBSERVED` only fires when `tshift` falls exactly on a node (rare: `tshift =
t − delay` is offset from the DT grid by the `delay`); honestly, most
frames are INTERPOLATED/EXTRAPOLATED (in the measured segment, 9800 extrapolations of ~31k). The
caller `place_causal` unpacks `p, prov` and uses `p`; `prov` is consumed by the assembly.

`vision_box` and `handoff` have their own provenance subset (Section 3.5): box
present → `OBSERVED`, coasting in a gap → `HELD`; handoff → `HELD` fixed.

## Decided boxes of `WaitingBoxLayer`

`draw()` retains `self.decided_boxes = {tid: BoxState(...)}` of the frame (the `cx,cy,ang,carril`
computed for painting are captured instead of discarded). `BoxState =
(tid, cx, cy, ang, anchor, carril, length, width, cls, prov)`, flat scalars (for the graph
blocklist). The anchor comes from the same drawing branch: `carril is not None` →
`"lane"` (homographic longitudinal + causal filter); otherwise → `"raw_homography"` (raw
homography + approach angle). `prov` = `OBSERVED` if the box was observed this frame, `HELD` if it is
coasting (in `vis` without observation). The heading travels as `heading_raw=(ang, "lane_angle")`
in the assembly.

**Verification:** the drawn polygons do not change, so the box shape readout
(`tr.polygon.getShape`, independent oracle) is byte-identical before and after (26820
lines, md5 `4c476519…`). Fidelity: `decided_boxes` reconstructs the real polygon (15438
checks, 0 discrepancies) and `anchor` is coherent with `carril`.

## `pixel_zones` occupancy

`occupancy_counts(fi)` (new method): the occupancy of all the zones of the frame, memoized
(`_occ_cache`), with a single call to `occupancy_counts` per frame, shared with
`_sur_c1_empty` (which now reads `["sur"]["lanes"][1]` from it). Zero parallel paths (single source,
it does not diverge by staleness). It is a raw count, not the paper metric (contract,
Sections 3.4/3.7).

**Verification:** `_sur_c1_empty` gives the same result as before, checked in two ways:
(a) the shape readout remains byte-identical (the c2-sur uses `_sur_c1_empty`, so
any change would move the polygons); (b) the memoized path matches the previous direct
computation per frame (3000 frames, 0 discrepancies).

## Handoff registry (`HandoffRegistry`)

Observed box→vehicle bridge (source `handoff`). Non-negotiable causal rule (only events
observed; it **never** queries `delay`/`cross_t`/`inst_t`):

- **Born** from `boxes_prev ∖ boxes_now` (the box of the root went extinct this frame) when its
  `vid` is not yet in `live`.
- **Dies** by: `vid ∈ live` (success), reappearance of the root in `boxes_now` (cancellation) or
  exceeding `handoff_max_hold_s` (presentation cap from config, not prediction).

`HandoffMark = (id=f"v{root}", source="handoff", last_pos, last_t, vclass, prov="HELD")`,
closed contract keys, no ETA. `last_pos` is the last observed position of the box.
The `{tid→root}` map collapses by root (re-identified fragments of the same root count
as one object); they are flat scalars, with no reference to `chains`.

`{tid→root}` comes from `orch.track_root` (exposed additively in `causal_core.py`:
`self.track_root = dict(track_root)` before the return; it does not change commits or chains → byte-identical
readout). It is re-id identity, not trajectory. `handoff_max_hold_s` = **2.0 s** in
`approach_zones.yaml → presentation` (an unpinned file, separate from `zones.yaml`).
Justified: ~5× the median instantiation lag (~0.37 s), a wide margin; most
handoffs die earlier by "vid appears" (< 0.5 s).

**Verification:** (a) 7 tests of the causal rule (`tests/test_vision_handoff.py`): birth
by extinction with observed `last_pos`, death by vid-in-live / box-reappears / cap,
does-not-be-born-if-vid-already-alive, collapse of fragments by root, and structural (the signature only takes
`tid_root` + `max_hold_s`, impossible to query the future). (b) Probing on real data (full segment,
5325 frames): 79 distinct bridges, 1319 frames with handoff, maximum 3 simultaneous,
0 violations (no handoff coexists with its live vid; all HELD/handoff/closed keys).

## Assembly: `from_vision_tick`

`ws_emitter/view.py`. It assembles the three sources into the `DecidedTickView` (the same dataclass
frozen as messages; `from_messages_tick` intact). New frozen entities: `BoxEntity`
(id, source, provenance, pos_plane, heading_raw, anchor, carril, vclass) and `HandoffEntity`
(id, source, provenance, last_pos, vclass; closed keys, no pos_plane/heading/ETA).

- **sumo_entities**: `sorted(sumo_subs)`, requires being decided (`vid ∈ sumo_prov`);
  pos/heading from the readback, provenance from `causal_pos`, `generation` always 0 in vision.
- **box_entities**: from the decided boxes; `anchor` lane|raw_homography;
  `heading_raw=(ang_rad, "lane_angle")`.
- **handoff_entities**: from the registry; `last_pos=(x,y,t)`, provenance HELD.
- **occupancy**: `kind="pixel_zones"`, `semantics="conteo_crudo"` (not the paper metric,
  Sections 3.4/3.7), immutable copy of the occupancy.

Canonical id `f"v{root}"` stable across source (box→handoff→vehicle, Section 3.3): the
caller (`--3d` phase) resolves `tid→root→id` and `cls→vclass`; the feeders arrive as
already-resolved scalars, the view touches neither the identity map nor `chains`. Zero retention:
everything is read and discarded; only str/int/float/tuples/MappingProxyType over
copies are retained.

Graph blocklist (`tests/test_vision_view.py`, anti-leak guarantee):

- **Positive:** sentinel baits in `veh`, `wb`, `v["draw"]`, `chains`, `inst_t`; the transitive
  closure of `gc.get_referents` of the view reaches no bait.
- **Negative control (detects when it must fail):** a view with an intentional fault that
  retains `veh` does name the leak (`"veh"` and `"draw"` that lives inside). Without this the test would not
  prove anything. The real view, with the same baits, leaks nothing.
- **Structural:** the signature has no `veh/chains/wb/draw/inst_t/tid_root` parameter
  (impossible to leak through the interface). The bait in `chains` covers the `{tid:root}` path: the
  view does not reach it because the id arrives already resolved as a string.

## End-to-end wiring

Transport reuse: `MessagesEmitter`, `WsBroadcastServer` and `canonical_bytes` are
shadow-agnostic, reused as is. The only specific part was `step_message`
(emitter.py), extended to the three sources + `occupancy` by `kind`:

- `sumo` → `{id,source,provenance,pos_plane,heading,vclass,generation}` (intact).
- `vision_box` → `{…,pos_plane,heading,anchor,carril,vclass}` (no generation).
- `handoff` → `{id,source,provenance,last_pos:{x,y,t},vclass}` (no pos_plane/heading).
- `occupancy`: `kind=="sumo_lanes"` → per_lane (messages); `"pixel_zones"` → per_zone +
  `semantics="conteo_crudo"` (vision).

**No-regression (strict verification):** the deterministic messages `ws_log` after extending
`step_message` is byte-identical (`f53c32d0…`, including the `end` line); the byte-exact pin of
`test_step_message_mapeo` is maintained. Extending did not touch the closed shadow.

**Heading converter `lane_angle`:** Section 3.6 fixes the contract convention as that
of `lane_angle_towards_stop` (rad, 0=+x east, CCW); the converter only normalizes the range to
[0,2π) (`rad % 2π`), verified against the contract, not from memory; pinned in
`test_vision_emitter.py`.

The caller (`build_vision_view` in `run_harness_causal.py`) assembles the scalar feeders
per frame from what is decided/observed, never from `veh[vid]`:

| Feeder | Causal source |
|---|---|
| `sumo_subs` | `getAllSubscriptionResults()` post-step (readback) |
| `sumo_prov` | `prov_out` filled by `place_causal` (branch of `causal_pos`) |
| `sumo_vclass` | **`getTypeID` post-step** (`VAR_TYPE=0x4f`) → `_VTYPE_VCLASS`; **NOT `veh[vid]["cls"]`** (the hole closed in messages) |
| `sumo_generation` | 0 (contract: vision always 0) |
| `boxes` | `wb.decided_boxes` (decided boxes); canonical id `f"v{tid_root[tid]}"`, vclass `COCO_VCLASS[cls]` |
| `handoffs` | `HandoffRegistry.update` (handoff registry) |
| `occupancy_pixel` | `wb.occupancy_counts(fi)` (`pixel_zones` occupancy) → `{zona:{carril:conteo}}` |
| `guard_report` | counters of `SondaTotal` |

`tid_root`/`COCO_VCLASS`/`_VTYPE_VCLASS` are identity and fixed maps (not `veh`). Minor work
in the core: `place_causal` gains `prov_out=None` (pure write) → `run_readout`
byte-identical.

**Vision hello (`vision_hello_fields`):** `shadow="vision"`,
`layers=["sumo","vision_box","handoff"]` (vision_box constitutive),
`occupancy_semantics={kind:"pixel_zones",semantics:"conteo_crudo"}`, `frame_geometry=null`
(base demo, no `bbox_px`), `vclass_dims` from the vTypes (single source), md5 of network and tracks.

`run_live` gains `ws_em=None`: it emits hello before the loop, `emit_step` after the pacing
(scheduled instant → realtime), `emit_end` on close. A single clock against the video (the pacing
`T0+t_v` already existed): ffplay plays the video externally, only the shadow loop is paced,
without the two-clock problem of mode 5; the video↔shadow offset is the instantiation `delay`
(~0.37 s, already in the HUD).

**Verifications:**

1. **Messages no-regression:** `ws_log` `f53c32d0…` byte-identical + `test_ws_emitter` (28) +
   `test_step_message_mapeo` (byte-exact dict). Messages profile intact.
2. **Vision passivity:** `run_readout` byte-identical (`04f3dd00…`/`8132571d…`) with `prov_out`
   and `VAR_TYPE` in the subscription (read-only). The emitter does not command SUMO.
3. **`lane_angle` verified** against Section 3.6 + `step_message` of the three sources
   (`test_vision_emitter.py`, 4 tests).
4. **Stream comes out:** headless probe with a real WS client — hello `shadow=vision` + layers +
   2600 steps with the three sources (`sumo`/`vision_box`/`handoff`) and
   `occupancy.kind=pixel_zones`; the client received hello+steps+end. The canonical identity is
   stable: the same `id` (`v0`) appears as box, handoff and sumo.
5. Full suite: 205/205. `VAR_TYPE=0x4f` verified against `traci.constants`.

## `--3d` command

`run_harness_demo_causal.py --3d`, additive to `--readout/--split/live` (mutually
exclusive group). It reuses almost the entire mode 5 of messages:

- **Lazy import** (`_run_web3d`): `ws_emitter.emitter` and the webserver are imported inside
  the branch, so the modes without 3D do not load `ws_emitter`/`websockets`. Verified in
  runtime: `--readout` ends with `WEB_EN_RUNTIME: NINGUNO`.
- **Webserver** (`ws_emitter.webserver`, a separate process without TraCI, direct reuse): serves
  `web/` + `/config.json` generated from `web3d.yaml`. The vision ports live in
  `web3d.yaml` (`vision_http_port: 8301`, `vision_ws_port: 8932`), different from those of
  messages/mode 5 (to run the two shadows at once); scene and host shared (same
  network). Config, not hardcode (override by `HTTP_PORT`/`WS_PORT`/`GRACE`).
- **URL and grace:** it prints `>>> SOMBRA 3D (VISIÓN): http://HOST:PUERTO/index.html` and does
  `sleep grace` before starting (the browser connects; ffplay and shadow start together after
  the grace). Retained hello → the browser that connects during the grace already receives it.
- **A single clock against the video:** `emit_step` hooks into the already-existing pacing of
  `run_live` (`target=T0+t_v`, after the sleep). ffplay plays the video externally;
  only the shadow loop is paced, without the two-clock problem of mode 5.
- **Cleanup (trap):** `atexit` + SIGINT/SIGTERM handlers kill the webserver; `run_live`
  closes SUMO and ffplay in a `finally`. Verified: after killing the command, `pgrep` of
  webserver/ffplay/sumo comes back empty.

### Video↔shadow offset

The reconstructed shadow runs ~`delay` (instantiation lag, ~0.37 s, already in the HUD as
"rezago de instanciacion") behind the video. It is the real latency of the reconstruction, a
result that must be seen, not an artifact to correct. No attempt is made to align video and shadow to
zero: doing so would hide the latency. (Unlike mode 5, where the problem was jitter
between two independent clocks; here there is a single clock paced against the externally played
video.)

### Command verifications

1. **Passivity:** `--readout` via the command → byte-identical readout (`04f3dd00…`), probe
   PASS. The emitter/webserver do not command SUMO nor change the reconstruction.
2. **Lazy import:** `tests/test_vision_lazy.py` (import) + real `--readout`
   (`WEB_EN_RUNTIME: NINGUNO`).
3. **Real command:** `python run_harness_demo_causal.py --3d` → printed URL + vision `/config.json`
   (`ws://…:8932`, `scene_rusia.json`) + stream served with the three sources
   (`sumo`/`vision_box`/`handoff`, hello `shadow=vision`, 924 steps in 40 s via a real WS
   client). ffplay and shadow start after the grace.
4. **Cleanup** verified (empty pgrep after killing). `from_messages_tick` and the mode 5 of
   messages intact. Suite 206/206.

## Location of `handoff_max_hold_s`

It is a presentation limit (how long the bridge ghost is drawn), not a measurement one. Its
correct home is the presentation config of the queue layer (`approach_zones.yaml`, which already
houses the box tracker parameters and declares its own life cycle, unpinned). `zones.yaml` is
measurement config and is pinned by the provenance guard; its pin must not be touched by
a parameter that is not measurement. Mixing the two layers by "semantic home" would be the
mistake: the semantic home of a presentation parameter is the presentation.

`zones.yaml` is an input pinned by md5 for reproducibility: the provenance guard
(`common.check_provenance`) aborts if it is touched (same as `site.yaml`). That is why
`handoff_max_hold_s` lives in `approach_zones.yaml → presentation` (a file declared
explicitly separate from `zones.yaml`, with its own life cycle of the queue layer), with the
loader `load_handoff_max_hold` that reads from there.

`build_veh_causal` returns a 4-tuple (includes `tid_root`); its 3 callers are
`run_harness_demo_causal.py` (the `--3d` one, which also passes `tid_root` to `run_live`) and
`validar_causal_completo.py` (×2).

## Passivity and provenance verifications

Two distinct closing conditions:

1. **Byte-compare (passivity).** `run_readout --video tramo` (which exercises
   `place_causal`→`causal_pos` and the subscription, and dumps via independent getters) before and
   after the changes:
   - `causal_sumo_readout_causal_completo_tramo.csv`: byte-identical (md5 `04f3dd00…`).
   - `causal_sumo_lanes_causal_completo_tramo.csv`: byte-identical (md5 `8132571d…`).
   - Identical TOTAL probe PASS (70 vehicles, 9800 causal extrapolations).
   The position did not change; neither the subscription nor the `(pos, prov)` perturbed the dynamics.
2. **Provenance branch test** (new datum the byte-compare does not detect):
   `tests/test_vision_provenance.py`, 6 tests, each branch of `causal_pos` exercised and the
   label verified against the branch taken, plus the check of literal TraCI
   constants vs `traci.constants`.

**Cross-cutting:** `DT`/`GAP_DT` come from `load_site_config("rusia").fps_ref`, not embedded;
nothing of a concrete video enters the code. The vision line runs with `.venv` (which has
`supervision==0.28.0`), not with the system `python3`.

## Known limitations

- The served client (`index.html`) is the messages one: the vision stream is emitted complete
  (the three sources and `occupancy.kind=pixel_zones`), but the specific render of the vision
  sources —queue by `anchor`, handoff representation, `pixel_zones` HUD, per-source composition
  of the vision layer— is not part of this module.

## References

- `ws_payload_contract.md`: v3 payload contract (sources, provenance, layers, Sections
  3.3–3.7).
- `decided_tick_view.md`: the `DecidedTickView` that this emitter builds and the anti-leak
  guarantee.
- `ws_emitter_impl.md`: the message emitter (`step_message`, `MessagesEmitter`, server)
  reused as is.
- `zones.md`: the zones layer and the exclusion; `approach_zones.yaml` for the queue
  presentation parameters.

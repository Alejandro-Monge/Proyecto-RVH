# DecidedTickView — the WebSocket emitter interface

An object that the digital-shadow emitter consumes instead of the process's raw state, so that it can only access data already decided at the present tick. It is coupled to the payload's v3.0.0 contract (`ws_payload_contract.md`): any later change comes with a new version of the contract. The view is built at the post-step point and its reference graph does not reach the future state that the process keeps in memory.

## Role and threat model

In the vision shadow, the process that replays holds the full `veh` dict in memory before the loop: whole trajectories (`v["t"]`, `v["draw"]`, `v["rt"]`), `cross_t`, `delay`, `t_exit_commit`, `exit_obs`, that is, the future of each vehicle. An emitter that consumes `veh` can leak the future to the client without anyone noticing (a heading taken from `draw[i+40]`, a handoff that announces how long it will last). The project rule is that the payload at t contain only what is decidable at t.

`DecidedTickView` makes that rule structural: the emitter does not receive `veh`, it receives an object built at the post-step point whose reference graph only reaches data already decided for that tick. The leak stops being a resisted temptation and becomes a state unreachable from the interface.

## Construction

The view is fed from different sources depending on the shadow; in both cases, from data that is already past at the instant of construction.

### Vision shadow (`run_harness_causal.py`, the `run_live` loop)

It is built once per frame, after `tr.simulationStep(t_v)`, from exactly four feeders:

1. **Frame command tap** (precedent: `RecordingSink`, `digital_shadow/harness/sink.py`; the pattern of recording orders already exists). It records what `place_causal` commanded this frame: which vids were added, removed, or moved, and the quality branch of `causal_pos` (observed, interpolated ≤2·DT, extrapolated, or frozen) per vid. A command that has been emitted is, by definition, a decision already taken.
2. **Return of `wb.trk.update(t, present)`** (`WaitingBoxTracker`): visible boxes with anchored position, voted class, and lane, plus the input `present`, which lets OBSERVED (present this frame) be derived against HELD (coasting) without touching internal state.
3. **Post-step TraCI reads**: position, angle, and `typeID` of the live vids (pattern demonstrated in `run_readout`). The post-step state is, by construction, past.
4. **The view's own handoff registry** (internal state, fed only by feeders 1 to 3 from earlier ticks): see the Handoff registry section.

In the messages shadow there is no dict with the future (decisions happen tick by tick after the guard), but the view is used the same way as a uniform emitter interface.

### Messages shadow (`causal_player.py`, post-step point)

It is built after the occupancy block of the logical tick, from: `applied_this_tick` / `last_applied` (commanded state and its native `provenance`), `getAllSubscriptionResults` post-step (lane, speed, position; the position requires the minor change declared in the contract), the eligibility sets (`active`, `parked`, `last_applied≠None`, the same rule as occupancy), `generation`, `assigned_type`, and the `occupancy_counts` dict already computed in-loop. All access to messages goes through `GuardedMessageStore`; the view does not touch the store.

## Input and output

The view is immutable after construction (the emitter reads, never writes). Fields:

| Field | Type | Source (per shadow) |
|---|---|---|
| `seq` | int | loop index (fi / i) |
| `t_sim` | float | post-step reported by SUMO (`getTime()`) in both shadows (single contract semantics). In vision it is a fresh read from feeder 3 (`SumoStage.sync` already returns it); the imposed logical clock remains derivable as `seq·dt` |
| `sumo_entities` | list of `{id, pos_plane, heading_raw, vclass, provenance, generation}` | vision: tap (live vids + quality branch) + post-step TraCI (pos/angle/typeID); messages: revealed eligibles + subscriptions + native provenance |
| `box_entities` | list of `{id, pos_plane, heading_raw, vclass, carril, anchor, bbox_px, provenance}` | vision only: return of `trk.update` + the present frame's row from the tracks CSV (bbox). `anchor` is derived from the same branch that decides the drawing (`WaitingBoxLayer.draw`): `carril is not None` → `"lane"` (lateral to the lane center + filtered longitudinal); `None` → `"raw_homography"` (no-lane fallback, raw position; the contract asks the client to draw it differently for visual validation) |
| `handoff_entities` | list of `{id, last_pos:{x,y,t}, vclass}` | the view's internal registry (see Handoff registry). Closed key set |
| `occupancy` | discriminated union of the contract (occupancy block) | messages: in-loop dict; vision: `zones.occupancy_counts` over the present frame's contacts |
| `guard_counters` | accumulated dict | messages: `store.guard_report()`; vision: tap counters + `SondaTotal` (n_extrap, max_interp_gap) |

Design notes:
- `heading_raw` carries the source and its origin convention labeled; the conversion to the contract's convention is done by the emitter's serializer at a single point.
- `vclass` of the SUMO vehicles comes from **TraCI (`getTypeID`) post-step**, not from `veh[vid]["cls"]`. This is deliberate: it is the only property that would tempt one to read `veh` "just this once", and closing that access removes the last path for reading `veh`.
- `id` is the contract's canonical one: the view derives it (vision: chain root; the `vid = "v{root}"` of `orchestrator.py:66` and the box's `tid` share the root).

## Handoff registry

Internal view state that persists between ticks (it is the view's only memory) and only records observed events:

- **Entry:** an `id` that was in `box_entities` at the previous tick and is not in the current one, and does not appear in `sumo_entities`, enters the registry with its last decided position (that of the last `trk.update` that included it) and its `t`.
- **Exit:** the `id` appears in `sumo_entities` (observed event "the vid was born in `live`") and leaves the registry. It also leaves if it reappears as a box (false handoff: a presence gap; the registry returns it to a box without noise).
- **Defensive expiry:** a resident that neither reappears nor is born in SUMO expires after `lost_track_buffer_frames·dt` (the R2 timeout already configured in `site.yaml`) and is reported in `guard_counters`; a handoff that never ends is an observable error, not a silent residue. The contract marks this value as an open client decision: R2 is several seconds and the real gap is ~0.37 s median, so the expiry value is decided when building the client, not here.

The registry never consults `cross_t + delay` or `inst_t`: it does not know when a handoff will end, exactly as a live pipeline would not know. That is why the contract can prohibit prospective fields without reservations: the information is not in the view.

## Deliberate exclusions

The object graph reachable from the view contains no reference to:

| Excluded | Why its presence would make the leak possible |
|---|---|
| the full `veh` dict | entire future trajectories (`t`, `xy`, `rt`, `draw`, `segs`) |
| `cross_t`, `delay`, `inst_t` | allow announcing when a vehicle will appear (handoff ETA) |
| `t_exit_commit`, `exit_obs`, `exit_edge_obs` | observed destination before its commit |
| `chains` / `commits` of the orchestrator | the complete already-resolved history, includes chain end |
| `_raw_by_frame` outside the present frame | occupancy and positions of future frames |
| `GuardedMessageStore` (messages) | direct access to messages; the view only sees already-applied state |
| `trackers[veh].states` (messages) | slots decided ahead of the application tick |

## Impossibility of a leak by construction

The argument: (1) the four construction feeders only produce data of already-executed commands, returns already computed for the present frame, and post-step reads; (2) the view's constructor does not accept `veh`, `chains`, or the store, there is no parameter to pass them; (3) the only memory between ticks (the handoff registry) is fed by the view's own earlier instances, by induction just as clean. Every emittable datum is, by chain of custody, decided past.

The limit: Python does not prevent a malicious emitter from importing the module and reading global `veh`. The layer-1 guarantee is **architectural** (the interface makes the leak impossible by the intended path and visible in code review as a flagrant violation), not cryptographic. Layers 2 and 3 cover that residue empirically: the emitter's probe asserts invariants over the emitted payloads, and the prefix audit (truncating artifacts and requiring byte-identical payloads in the common prefix) detects any dependence on the future, come from wherever it may. Layer 1 makes the leak hard and unreadable; layer 3 makes it detectable with experimental certainty.

## References

- `docs/modules/ws_payload_contract.md`: the v3.0.0 contract that the emitter consumes (the view is its source interface).
- `docs/modules/ws_emitter_impl.md`: the emitter that reads this view.
- Pattern precedents in the repo: `RecordingSink` (order tap), `GuardedMessageStore` (guarded access), `SondaTotal` (invariants with verify at closing).

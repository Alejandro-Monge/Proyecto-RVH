# Module — Reconstruction in SUMO by lane geometry

Reconstructs each vehicle's trajectory over the clean geometry of the SUMO net, which is where
the shadow lives and where the results are read from. The previous reconstruction used
`moveToXY(raw H projection)`, which ended up **~1 lane offset per zone** and forced the route to the
**predicted** destination (sometimes wrong: the west→north bus 1 froze at the east junction).
This module separates **shape** and **advance** to avoid both problems. The code lives in
`digital_shadow/sumo/reconstruct.py`; it is used by `scripts/run_harness_viz.py` (screencast) and
`scripts/run_harness_demo_causal.py` (live demo), and its verification is visual.

## Shape and advance

- **Shape = a continuous net path** (`build_path`): classified entry lane → internal lane(s) of the
  **observed** turn (traversed completely with `_internal_chain`, including the `via` of the left
  turns) → egress lane (successor of the internal). Concatenated end-to-start and resampled to ~1 m.
  **The approach→junction→egress transitions are smooth by construction** (a single path; without the
  ~1 lane jump of the previous patch).
- **Advance = the forward component of the observed displacement** (`reconstruct_world`): per frame,
  `Δs = (obs[i]−obs[i−1]) · tangente_del_path`. The lateral jitter of the ground contact cancels
  (perpendicular to the tangent) and there is no curve bias (unlike projecting the absolute position,
  which advances on the outer side of the turn). **It respects stops** (if the trajectory does not
  advance, `Δs≈0`). Ground-contact pre-smoothing (median v3) plus median of the advance (v5) for the
  residual jitter. **No α-β velocity filter** (it would extrapolate during the stop).
- **Observed destination** in the complete ones (95%, `run_core`); predicted / approach-only in the 4
  partial ones.
- **`keepRoute=2`** (reverts the regression to `keepRoute=1`).
- **Grouped lanes** (west 2-3, the camera does not separate them): **egress disambiguation** (a general
  geometric mechanism, in `run_core`, not a west table): in the complete ones, the observed turn
  restricts the lane of the group — **left → leftmost lane** (maximum, because 1=right), **right →
  rightmost** (minimum); the turn is obtained by inverting `turns[(entry,exit)]`. The **straight does
  not disambiguate → lateral mean** of the group (declared sensor limit, not a bug). Causal
  implication: the recovered lane is observed information (high delay, on exit); in the drawing it does
  not matter, in causal measurement it will be an observed commit. Example: the west→north bus (left
  turn) → lane 3.

## Input and output

`build_path(traci, lane_map, entry_arm, exit_arm, carriles) → path (Nx2)`.
`reconstruct_world(raw_xy, ts, path) → recon_xy (Nx2)`. The render places `moveToXY(recon_xy, keepRoute=2)`.

## Limits

- **Over-advance in tight curves ~0.5–0.6 s:** the observed one, on the outer side of the turn,
  traverses a longer arc than the centerline, so the accumulated advance runs a bit ahead. Net with
  the delay (~0.4 s), the vehicle ends up almost aligned. It is a known geometric limitation.
- **Residual spikes ~1 m** (≈29/80 vehicles): raw ground-contact jitter at the entry→internal seam of
  the misaligned grouped ones and in the gaps; they are minor and do not break the drawing.
- **It does not include buffer reconciliation** in the gaps (that corresponds to the reconnection
  layer): a gap without reappearance is covered with the render's linear interpolation.

## Unified connection: drawing by observed entry and exit lanes

`choose_connection(traci, lane_map, entry_arm, exit_arm, carriles, exit_carril=None)`: a single net
connection choice, shared by the reconstruction (`build_path`) and by the render routing
(`departLane`). This way the entry, the internal curve, and the egress always come from the same lane,
and the sim route coincides with the recon. It replaces the previous design (`_choose_connection` with
two conditions plus a companion and a fixed `_mean_entry_shape` branch), which were three symptoms of
the same root. Logic:

1. **Connection of the observed entry lane**; among those, that of the observed exit lane
   (data-dependent egress). It covers legal and routable (including the real GT violations that the
   net-edit added: north c1→south `:J5_18`, north c2→east `:J5_19`).
2. **Not routable** (no connection leaves the observed lane → an impossible tuple in the net =
   unreliable observation): it falls to the **complete canonical connection** of the movement (`via =
   junction_internal`): entry + internal + egress of the same canonical lane, without a jump. It is the
   maneuver without knowing the future or the GT; here `exit_carril` is not used (it comes from the
   same unreliable observation). That it coincides with the GT in this segment is a reported result,
   not the design criterion.

> **Why non-routable routes do not respect the observed exit lane.** The case that tempted it
> —the south→west exits that the canonical fallback puts in lane 1— comes from an erroneous annotation
> of the GT, not from SUMO: south→west turns exit through **lane 1** (the GT annotated them as lane 2
> by an automatic "left-turn→left-lane" rule). SUMO already draws the real lane (1) by the canonical
> fallback; respecting the observed exit lane would force lane 2 (the wrong value). The canonical
> fallback is kept. (Not to be confused with the "Option B" of the zones/lane design in `zones.md`.)

- **`departLane` is derived from the chosen connection** (`from_idx`): in legal/routable = observed
  lane; in the non-routable case = canonical lane. The render passes it to `stage.add` (before it was
  `departLane=0` for all), so recon and routing share the connection and the recon-vs-sim mismatch
  (~3 m at the junction entry) closes to ~0. It is never instantiated on the raw observed one when they
  differ, so the jump of the `_mean_entry_shape` + canonical internal of another lane fallback is not
  reintroduced.
- **Net-edit prerequisite** (additive, does not touch coordinates): 2 connections for the GT
  violations (linkIndex 16/17) + the `dir` of `:J5_19` corrected to `l`. Homography intact (it comes
  from control_points).

Verification (read from SUMO after `simulationStep`, trajectory clips): 52 legal recon
byte-identical (0 regression, proven by hashes) + 18 routable identical; 10 non-routable fall to the
canonical one without teleport; exit lane ≥ baseline. Note: `keepRoute=1` produces a visual jump ~5 m
north c3→east (v34/v65) in the internal curve of the junction — SUMO snaps to `:J5_19` (the c2 internal
that the net-edit left parallel to `:J5_14`, converging to the same egress) because the route is
edge-level and does not fix the internal one. **Minor visual limitation:** no impact on metrics (entry/exit
lane, presence, delay correct); the 3 fixes (hint/kr2/net) were discarded with data. It is the
contained price of completing the net with the real c2→east route.

## Isolation

It does not touch the measurement: `measure_harness.py` keeps running with `RecordingSink` and reading
from the core, with the suite 71 intact. Reorienting the metrics to read from SUMO corresponds to the
measurement stage.

## References

- `docs/modules/sumo_lane_map.md`: the approach/lane↔edge/lane mapping and `junction_internal`.
- `docs/modules/sumo_stage.md`: the scenario that runs the render (`moveToXY`, `add`).
- `docs/modules/zones.md`: the zones/lane design (Option B).
- `docs/modules/causal_reconnect.md`: the buffer reconnection layer.
- Site config: `config/sites/rusia/sumo/lane_map.yaml`, net
  `config/sites/rusia/sumo/intersection_rusia.net.xml`.

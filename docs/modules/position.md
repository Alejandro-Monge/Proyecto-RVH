# Module — Position piece of the SUMO shadow (`digital_shadow/sumo/position.py`)

Decides **where to place each vehicle replicated in SUMO**: `(laneID, pos_longitudinal)` for
`traci.vehicle.moveTo(vehID, laneID, pos)`. The architecture is **position-driven** (the shadow places
the observed position; SUMO does not simulate the movement), because delay and continuity are measured
over what the shadow observed and represented.

The position is **decomposed by axes** (a data-audited decision, see the validation below):

- **Lateral = lane (discrete).** The classified lane (image plane, at the stop line) is
  translated to a SUMO lane via `lane_map`; SUMO **centers** the vehicle in that lane. The homography
  is **not** used for the lateral. It is committed at the **stop-line crossing** (causal instant).
- **Longitudinal = homography + causal filter.** The ground contact is projected to world coordinates
  and projected onto the **approach edge's axis** (from the net) to give `pos`; it is smoothed with a
  constant-velocity causal filter (short memory, past only).

## Input and output

- **Pure transform** (stateless): `project_to_world(H, px)` (homography, per point) ·
  `longitudinal_pos(axis, world_xy)` (projection onto the axis, clip to `[0, length]`) ·
  `raw_pos(H, axis, px)` (composition) · `resolve_lane(lane_map, arm, label) → (laneID, mode)`.
- **Past-only state:** `AlphaBetaFilter` (longitudinal filter) and `VehiclePositioner` (per
  vehicle). `VehiclePositioner(arm, lane_map, H, alpha)`: `observe(t, contacto_suelo_px)` each
  frame (feeds the longitudinal filter from birth), `commit_lane(label)` at the crossing
  (fixes the laneID), `placement() → (lane_id, pos, lane_mode)`. `lane_id` is `None` until the crossing;
  `pos` is available from birth. The **when** (when it crosses, when it instantiates) is
  decided by the **harness**, not this piece.

`resolve_lane` modes: **exact** (lane resolved); **grouped** (west {2,3}: representative = lane
2, the rightmost, sensor limit); **fallback** (arm_only or unresolved → lane 1, the
rightmost, declared limit, for example the south; **not** a defect).

## Longitudinal filter — constant-velocity alpha-beta

`config/positioning.yaml: longitudinal_filter`. The model is constant velocity with approximately
stationary noise (world jitter 0.06–0.16 m RMS, homogeneous per approach); alpha-beta is
the **fixed-gain** form of the constant-velocity Kalman in the stationary regime, so it
gives the same smoothing with less machinery: it needs no per-frame covariance (only `pos` is
pushed) and it handles small N without tuning Q/R. Kalman remains an improvement **only** if
per-region uncertainty is needed later (weighting by the homography error, which varies per approach).

`alpha` (position gain) comes from config; `beta = alpha²/(2−alpha)` is derived (critically
damped). With **`alpha = 0.4`** the effective memory is ≈ 4 frames ≈ **0.13 s** (sub-second).

That memory (~0.13 s) is **not** the reconnection buffer (`lost_track_buffer_frames` = 140 f ≈
4.73 s). They are different things: the buffer waits for evidence to join fragments (continuity); this
filter smooths the position jitter. **The `pos` must not be averaged over the buffer**: a simple
average over 140 f would bias the position meters backward (measured).

## Causal fit

It satisfies the contract (`docs/modules/causal_contract.md`): the transform is a **pure** lookup
(clause i); the filter and the positioner are **past-only** state (clause ii), verified with
`check_backward_only`. **Longitudinal and lateral are independent and both causal:** the longitudinal
filter runs from birth over the approach axis (the approach is known early); the
lateral (lane) is committed at the crossing, an exact reflection of the decomposition. Without
anticipation (it does not look at future frames) there is no extra delay cost: the only delay is the
reconnection buffer, already declared, plus the ±3 frame window of the lane ≈ 0.1 s.

## Data validation

- **The advantage of decomposing (measured):** the per-lane discretization **discards the lateral
  error of the homography**. In the LOO decomposition of the H error per axis
  (`scripts/analyze_homography_loo_axes.py`), the lateral is **~47 % of the error energy** (up to
  **86 % in the east**) and **does not reach SUMO**. The `pos` only drags the **longitudinal**: LOO
  longitudinal per approach, east **0.31 m**, north **1.13 m**, south **1.21 m**, west **4.52 m** (this
  last one is the homography horizon limit, already documented; this piece does not create it).
- **Robust lateral:** the per-frame lane classification near the crossing **never changes lane**
  (0 % changes where the lane is resolved; the south is arm_only → declared fallback).
- **The filter reduces the jitter without lag:** over real tracks, the longitudinal acceleration noise
  drops **~65–67 %** (north 0.097→0.033, east 0.108→0.035, west 0.169→0.060, south 0.146→0.049 m); at
  constant velocity the lag is ≈ 0. A simple average over the buffer would bias **1.6–6.6 m**.
- `alpha = 0.4` validated against a synthetic signal (best noise reduction with lag ≈ 0) and real
  tracks.

## Scope and limits

- **Scope of this step:** the **entry** side (approach). The egress and the junction are composed in
  the **route** piece (`docs/modules/route.md`).
- **Horizon limit (west):** the longitudinal accuracy is bounded by the homography (west
  4.52 m, single-plane breakdown). The decomposition prevents the drift from adding up and eliminates
  the lateral, but the longitudinal limit is the H. Documented, not hidden.
- **South arm_only:** placement at the approach level (default lane), sample limit already declared.
- **Minor approximation:** the `pos` is measured over the lane 0 axis (the approach lanes are
  parallel and of equal length → the `pos` is the same for all, cm-level error). It remains to be
  confirmed with SUMO running.

## Files

- `digital_shadow/sumo/position.py` — the piece (pure transform + filter + positioner).
- `digital_shadow/sumo/lane_map.py` — `approach_axis` (longitudinal axis per approach, derived from the net).
- `config/positioning.yaml` (alpha), `config/sites/rusia/site.yaml: homography_ref`,
  `config/sites/rusia/sumo/lane_map.yaml: approach_axis`.
- `scripts/validate_position_piece.py`, `scripts/analyze_homography_loo_axes.py` — validation.
- `tests/test_position.py`.

## References

- `docs/modules/causal_contract.md`: the causal contract that the piece satisfies.
- `docs/modules/route.md`: the route piece that composes the egress and the junction.
- `docs/modules/sumo_lane_map.md`: the lane map and the approach axis.
- `docs/modules/ground_contact.md`: the reference point that is projected.
- `docs/modules/lane_classifier.md`: the entry lane used in the lateral.

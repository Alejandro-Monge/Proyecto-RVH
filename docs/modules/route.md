# Module — Route piece of the SUMO shadow (`digital_shadow/sumo/route.py`)

Decides the vehicle's **route**: **origin** (where it entered) and **destination** (exit approach or
edge), for `traci.vehicle.changeTarget`. The design was audited with data (see the validation below);
the finding that governs it is that the **direction derived from the trajectory is useless in the
south** (38° of jitter), but the **entry lane predicts the turn** with 88–100 % —and 100 % in the
south, in the GT. The decisive factor is the lane, not the direction.

## Input and output

- **Origin:** committed at the **stop-line crossing** (`entry_arm` + `entry_lane`), causal instant.
- **Destination — three paths, with recorded evidence** (not metadata: it is the measurement of the
  delay-vs-accuracy curve):

| Path (`evidence`) | When | Signal | Delay |
|---|---|---|---|
| `predicho_carril` | at the stop crossing | `carril→giro` (rule calibrated from the GT) → `exit_arm`/`exit_edge` | low |
| `observado` | at the exit crossing | full trajectory enter-curve-exit | high |
| `arm_only` | at crossing, **south** | no resolvable lane → no turn prediction (declared fallback) | low |

**Pure** lookups: `predict_turn(rule, arm, lane)` → turn or `None`; `predicted_exit(...)` → `(turn,
exit_arm, exit_edge)`. **Causal** state: `RouteCommitter` emits **sealed and append-only** commits.

## The south lane

The south falls to `arm_only` **not because GT is missing** —the south GT is complete, all 14 with a
labeled lane— but because **the classifier does not read the south lane from the image with
confidence** (10/14 measurable at the stop, wide CI → `arm_only`, already declared in the classifier).
Moreover, the turn prior of the south approach is **almost uniform** (5 right / 5 through / 5 left), so
there is no reliable default. That is why the south **abstains** from predicting a destination (evidence
`arm_only`); its gap filling falls to straight extrapolation (declared limit, not a defect).

## Sealed deferred commit (contract clause ii extended)

`predicho` and `observado` are **two distinct, sealed, append-only commits**: the commit at *t* **is
not rewritten** by later evidence; what arrives on exit is a **new** commit with its seal. Verified
with `check_deferred_commit` (`digital_shadow/causal`), which catches any rewrite. `RouteCommit`
(frozen) carries `evidence`, `commit_time`, `entry_time`, `delay`, `exit_arm/edge`.
`RouteCommitter.prediction_hit()` compares predicted vs observed (accuracy for the delay-vs-accuracy
curve).

## Data validation (4 blocks)

1. **Lane→turn accuracy vs observed destination (end to end, classifier lane):** north
   **100 %** [83,100], east **95 %** [76,99], west **88 %** [64,97] (Wilson 95% CI); **south abstains**
   (14, arm_only). Maximum of the rule with GT-lane: 88–100 % LOO.
2. **changeTarget against running SUMO + pos (lane 0 axis) — later verification of the previous
   step:**
   - `changeTarget('-E1')` from `E5` → route `('E5','-E1')` **correct** (implicit `:J5_*` junction,
     connection validated 12/12).
   - **The pos (lane 0 axis) is faithful against real SUMO: error 0.026 m** (and 0.001 m vs SUMO's lane
     geometry). The lane 0 axis approximation is **confirmed**.
   - **Finding** (not in the audited design): with SUMO's control active, `moveTo` + route makes SUMO
     **change the vehicle's lane** (error 3.2 m ≈ 1 width). **Position-driven requires disabling SUMO's
     autonomous control** (`setLaneChangeMode(0)` + `setSpeedMode(0)`); then `moveTo` is faithful
     (0.026 m). It is a harness requirement.
3. **Corner cut that the route-based filling avoids** (junction curve vs straight line): left
   **8.34 m** (`:J5_6` south→west), **8.16 m** (north→east), right **6.38 m** (south→east), **5.86 m**
   (east→north), straight **0.00 m**. A straight line in the gap cuts several meters of corner;
   following the internal curve avoids it.
4. **Delay-vs-accuracy curve** (two extremes per approach): **predicted** delay ≈ 0 with the accuracy
   of block 1; **observed** delay = junction transit (≈ 4.9–7.6 s) with 100 % accuracy. The piece
   records the evidence per vehicle, thereby generating the curve.

## Gap filling

For a faithful position, SUMO's control must be **disabled** (block 2), and with the control disabled
SUMO **does not** advance the vehicle. Therefore, gap filling **cannot** be delegated to SUMO's
car-following: the position must be **interpolated along the shape of the predicted internal lane**
(given by the net, for example `:J5_6` for south→west), at the last observed speed, with the control
always disabled. `changeTarget` remains useful for the logical route and the O-D, but the **position**
in the gap is driven by us. The alternative of alternating the control (active in gaps, inactive in
observed frames) mixes SUMO's speed model and is less faithful. This filling corresponds to the
**harness**.

## Scope and limits

- **South:** abstains from predicting a destination (`arm_only` lane + uniform prior). The filling of
  its 3 junction gaps falls to straight extrapolation (accepted corner cut) until the south lane is
  resolved.
- **Omissions (4):** out of scope (no trajectory) → recall limit, not inference limit.
- **Real work ≈ 11 junction gaps** (8 with a lane signal, 3 of the south without), bounded, not
  large-scale.
- The **trajectory direction does not** enter as a turn signal (noise, especially in the south).

## Files

- `digital_shadow/sumo/route.py` — pure lookups + `RouteCommitter` (sealed commits).
- `config/sites/rusia/lane_turn_rule.yaml` (derived from the GT), `turns.csv`,
  `config/sites/rusia/sumo/lane_map.yaml`.
- `scripts/derive_lane_turn_rule.py`, `scripts/validate_route_piece.py`.
- `tests/test_route.py`.

## References

- `docs/modules/causal_contract.md`: the causal contract (deferred commit) that the piece satisfies.
- `docs/modules/position.md`: the position piece that provides `pos` and the lane axis.
- `docs/modules/lane_classifier.md`: the entry lane that predicts the turn.
- `docs/modules/sumo_lane_map.md`: the SUMO lane and edge map.

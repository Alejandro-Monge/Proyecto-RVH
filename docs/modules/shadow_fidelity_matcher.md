# Module — GT↔shadow matcher and fidelity metrics (`scripts/evaluate_shadow_fidelity.py`)

Measurement layer that matches each shadow vehicle to its ground truth event and compares the
fidelity dimensions. It is the yardstick: it does not touch `reconstruct.py`, the render, or the net;
the shadow looks the same and this only evaluates it. The code is in `scripts/evaluate_shadow_fidelity.py`
(test in `tests/test_shadow_matcher.py`).

## Motivation

The GT is event-level (per real vehicle: entry and exit approach and time, lane, class). To
measure fidelity you have to match each shadow vehicle to its GT event and then compare. A greedy
matcher by entry time **crosses identities** when two vehicles enter almost at the same time through
the same approach (24/65 pairs mismatched when comparing entry alone versus entry plus exit). While
the matcher crosses identities, no figure (presence, lane) is reliable. This module redoes it correctly.

## Method

It matches shadow↔GT and produces **four dimensions** of fidelity (LOO per vehicle, only over
complete-match):

1. **Presence** of each vehicle: complete-match / partial-match / partial-incierto /
   track-degenerado / duplicado-fantasma / fantasma-genérico / pérdida-GT.
2. **Lane** of entry and exit versus the GT, per approach (perception limit).
3. **Delay** of crossing versus the GT at pare1/pare2, per approach (time is fidelity: crossing
   delay is a dimension in its own right).
4. Strict one-to-one (0 GT with 2 vehicles, 0 vehicles with 2 GT).

Source: causal chains from `run_core` (the same as production; `build_chains` replicates its setup
and runs the orchestrator). Each crossing comes from the vehicle's own trajectory; the time or
crossing of another is never copied.

## Algorithm

Two stages plus quarantine, in `match(sigs, gt)`:

```
0. track-degenerado: completes with TRANSIT TIME (pare1→pare2) > TRANSIT_THR → quarantine.
1. completes × GT (Hungarian): same entry approach ∧ |Δt_in|≤W_IN ∧ same exit approach ∧ |Δt_out|≤W_OUT.
   Cost=|Δt_in|+|Δt_out|.                                                         → complete-match
2. RELIABLE partials (end_speed≥SPD_THR ∧ stability≥STAB_THR) × residual-GT: entry approach ∧ |Δt_in|≤W_IN
   ∧ geometric heading==GT-exit-approach.                                         → partial-match
3. UNCERTAIN partials × residual-GT: entry approach ∧ |Δt_in|≤W_IN (NO heading).  → partial-incierto
4. unmatched GT = loss. Unmatched system track: **COEXISTS in trajectory** with a matched one (overlap
   >DUP_OVERLAP_S ∧ mean dist <DUP_DIST_PX) → duplicado. Otherwise, the ghost is split by the TYPE of
   failure: it crosses a GT **LOSS** (same approach ∧ |Δt_in|≤W_IN) → **real-sub-trackeado** (RECALL
   failure: a real vehicle whose GT event exists, as a loss); if it crosses no loss → **espurio**
   (PRECISION failure: track with no real vehicle, or redundant extra alongside a matched one). The
   crossing uses ONLY approach+time, NEVER lane/class.
```

Completes first (two stages), not a global Hungarian: a **structural** guarantee that a partial never
steals the GT from a complete, with no penalty to tune. It recovers the police car as `partial-incierto`
(police-car entry detected, it fragments, it does not complete the route) instead of losing it.

## Parameters

Justified by data from the segment, not arbitrarily:

| parameter | value | justification |
|---|---|---|
| `W_IN` | 1.0 s | stable zone of the entry-window sweep (actual residuals ≤0.91 s); matches `matching_tolerance_s` |
| `W_OUT` | 1.25 s | start of the stable zone [1.25,1.5]; captures the real match v71↔ev26 (residual 1.17 s), excludes the 1.59 s outlier |
| `SPD_THR`, `STAB_THR` | 1.5 px/fr, 0.55 | gaps in the heading distribution (end_speed velocity and stability consistency); the stopped police car falls into uncertain |
| `TRANSIT_THR` | 20 s | point in the 10→42 s gap (the pathological one ~42 s; the rest ≤10 s). Duration or frozen do not work (there are valid matches just as long or frozen). The script reports the degenerate one(s) dynamically (not a fixed id); currently it is `v9` |
| duplicado | trajectory coexistence <25 px, >2 s | the distinct nearby ones go to box-IoU~0 / center-dist ≥28 px; center margin fragile (3 px), which is why coexistence is used, not entry Δt |

## Anti-leak restriction

The cost functions read **only** the physical approach, the pare1/pare2 crossing time and, for
partials, the geometric heading. **Never** `entry_lane_label`/`exit_carril` (the lane, which is the
reported metric) or the class (metric 7.5). The approach is used as an **anchor** (its errors produce
a ghost or a loss, not a mismatched pair), as production already did with the entry. This is offline
evaluation: using the exit instant (future) to match GT↔prediction is standard in tracking evaluation
(it does not violate the causality of the shadow, which already ran).

## Input and output

- **Input**: `verify_out/tracks_rusia.csv`, `verify_out/events/rusia/track_events.csv` (for the
  `excluded` set), `data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv`. SUMO
  only for the optional SUMO verification of the north→east teleport (vehicle looked up dynamically,
  previously fixed to `v44`; `--no-sumo` omits it).
- **Output**: console report (presence, lane, delay, changes versus greedy, one-to-one) and
  the CSVs `verify_out/events/rusia/shadow_gt_matches.csv` (category, lane and delay per vehicle) and
  `shadow_gt_losses.csv`.

## Dependency on regenerable artifacts

An upstream change (detector or tracker) renumbers the `track_id`s and regenerates `tracks_rusia.csv`,
`track_events.csv` and `reconnected/matches.csv`. If any is left unregenerated, its old ids
collide numerically with the new ones and produce wrong GT references. The invariant "52 LEGAL
recon byte-identical" applies to **code** changes (same tracks), not to **detector** changes
(which benignly perturb the trajectories). For upstream changes, the correct invariant is:
lane, presence and delay **no-worse**.

## References

- `docs/modules/track_events.md`: event-per-track layer and the `excluded` set.
- `docs/modules/matching_sombras.md`: matching core and shared metrics.
- `docs/modules/tracking.md`: source of `tracks_rusia.csv`.

# turns.csv — turn derivation table (Rusia site)

Maps each observed `(entry_arm, exit_arm)` pair to its turn direction and to its
feasibility/legality status. It is filled in by hand by someone who knows the
intersection. The pipeline does **not infer** the turn: it derives it from this
table, which is the single source of truth. The human who labels the ground truth
also does not annotate the turn.

## Columns

- `entry_arm`, `exit_arm`: the arms of the pair. Valid values: `norte`, `sur`,
  `este`, `oeste`.
- `turn`: geometric direction of the movement. Values: `left`, `through`,
  `right`, `u_turn`. It is left **empty** only when the movement is
  geometrically impossible (`status = impossible`).
- `status`: feasibility and legality of the pair. Values:
  - `allowed` — feasible and permitted by the signage.
  - `prohibited` — physically feasible but prohibited by the site's signage.
    **It has a turn direction: fill it in `turn`.** If it shows up in the
    labeling it is a **real violator**: a valid event that enters the metric and
    is reported as a finding, not an error.
  - `impossible` — geometrically impossible. `turn` is left empty. If it shows up
    in the labeling it is a **sure labeling error** (a QA bug): it is corrected and
    excluded.

This separation between `turn` (direction, a geometric attribute) and `status`
(feasibility/legality) is deliberate: it preserves the direction of the real
violators instead of collapsing them into a single "invalid" value.

## Filling rule

1. For each feasible pair, write its `turn` (`left`/`through`/`right`/
   `u_turn`) and set `status` = `allowed` or `prohibited`.
2. For each impossible pair, leave `turn` empty and set `status` = `impossible`.
3. **Do not delete rows**: the 16 combinations stay in the table so that the
   format is stable and the coverage is complete (it includes the same-arm pairs,
   candidates for `u_turn`).

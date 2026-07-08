# Module — Fragment reconnection by domain (`scripts/reconnect_fragments.py`)

Merges tracks that are **fragments of the same vehicle** (split by occlusion) using **domain**
signals, not appearance ones (the OSNet appearance re-id is disabled: the generic model is the main
limitation, addressed in the tracking layer). It operates at the **track** level: it reassigns
`track_id` (merge) and rewrites the tracks CSV; over that CSV the evaluation pipeline
(`derive_track_events` → `match_events_to_gt`) is re-run without touching the event layer or the
previous baseline, which are the reference. It attacks the deficit of the **south** approach of the
baseline, where fragmentation causes ghosts and omissions.

## The decision conditions (all must pass)

For a directed candidate **A.tail → B.head** (A ends, B begins):

| Condition | Rule | Threshold source |
|---|---|---|
| **Temporal** | `0 < gap ≤ buffer`. The `gap>0` requires B to start after A ends → excludes co-temporal duplicates (e.g. tid70/tid13, which overlap). | buffer from `site.yaml: reid_temporal_window_frames` (140 f ≈ 4.73 s); swept via CLI |
| **Spatial** | `dist(cola_A, cabeza_B) ≤ corridor` (px). Anti-teleport in magnitude. | `site.yaml: reid_spatial_corridor_px` (250) |
| **Kinematic** | the jump `cabeza_B − cola_A` continues forward: angle ≤ `MAX_HEADING_DEV_DEG` with A's heading at its tail (estimated over `HEADING_FRAMES`). If the heading is indeterminate (very short or stopped track), it is not rejected by angle. | script constants: 60°, 5 frames |
| **Class** | same majority class (vote over `class_id`). Avoids merging car↔bus. | — |
| **Route** | the movement of the merged track (entry of the 1st fragment, exit of the last) ∈ allowed movements of J5. | `config/sites/rusia/turns.csv` (`status == 'allowed'`) |

**One-to-one** greedy assignment by cost (`gap + dist·0.001`, tail and head used at most once) +
**union-find** (allows chains A→B→C). Bias toward **not merging** under ambiguity.

## Anti-over-merge criterion (the key piece)

Never merge a component with **≥2 complete cross-arm fragments** (entry and exit in real and distinct
approaches).

**Why (geometry):** a real crossing split in time produces at most one corner-to-corner stretch (the
1st piece goes entry→middle; the 2nd, middle→exit; neither alone covers corner-to-corner unless they
are two distinct vehicles). Without this criterion, the 1st run made **2 false merges** (`[76,99]`,
`[115,122]`, both `cross=2`, merging two vehicles that matched distinct GTs); with the criterion, **0
false merges** and all recoveries are preserved (which have `cross≤1`).

## Input and output

**Input:** `verify_out/tracks_rusia.csv`; `site.yaml` (image center, `arm_set`, buffer,
corridor); `zones.yaml` (exclusion + `stop_lines`); `turns.csv` (valid routes);
`control_points.csv` (contrast only, via the event layer).

**Output** (`verify_out/events/<id>/reconnected/`): `tracks_reconnected.csv` (tracks with merged
`track_id`), `matches.csv`, `baseline_por_brazo_reconnected.csv`, `buffer_sweep.csv`. The
script prints the before/after comparison per approach, the negative-case check, and the buffer
curve.

The **excluded** tracks (parking lot) are discarded as candidates (parked ones are not reconnected).

## The buffer and its sweep

The buffer = how long future evidence is awaited to confirm the continuation of a fragment
(analogous to the latency with which the V2C shadow fills lost packets). Sweep [1,6] s
(`buffer_sweep.csv`):

| buffer s | merges | matched | precision | recall | false merges | median delay s |
|---|---|---|---|---|---|---|
| 1.0 | 3 | 66 | 0.835 | 0.904 | 0 | 0.41 |
| 2.5 | 7 | 68 | 0.850 | 0.932 | 0 | 1.15 |
| 3.0 | 14 | 69 | 0.852 | 0.945 | 0 | 2.23 |
| 4.5 | 14 | 69 | 0.852 | 0.945 | 0 | 2.57 |
| 6.0 | 17 | 69 | 0.852 | 0.945 | 0 | 2.57 |

**Stable recall zone from buffer ≈ 3.0 s** (the south recoveries have gaps of 2.6–3.4 s);
**0 false merges over the whole range**; the introduced delay grows with the buffer. The config uses
the buffer from `site.yaml` (140 f ≈ 4.73 s), in the stable zone.

## Result (buffer 140 f ≈ 4.73 s, matching window 1.0 s)

15 accepted merges (35 tracks). **Negative case tid70+tid13 merged = False.** **False
merges = 0.** Before vs after per approach (from `baseline_por_brazo*.csv`):

| approach | recall before→after | precision before→after | exit accuracy before→after |
|---|---|---|---|
| north | 1.000 → 1.000 | 1.000 → 1.000 | 0.947 → 0.947 |
| **south** | **0.800 → 0.933** | **0.632 → 0.737** | 0.917 → 0.857 |
| east | 0.909 → 0.909 | 0.800 → 0.800 | 0.950 → 0.950 |
| west | 0.882 → 0.941 | 0.938 → 0.889 | 1.000 → 1.000 |
| **TOTAL** | **0.904 → 0.945** | **0.835 → 0.852** | 0.955 → 0.942 |

South counts: matched 12→14, **ghosts 7→5**, **omissions 3→1**.

**Scope:**
- **Classification** (prior cross with `fragmentation_events_baseline.csv`): **6 of the 7**
  south ghosts of the baseline `[70,72,74,85,96,98,117]` are of the **fragmentation type** (5
  cataloged + tid98); the 7th (**tid70**) is a **co-temporal duplicate** (see «Limits»).
- **Effect of this config** (not to be confused with the classification): the reconnection
  **eliminated 2 south ghosts** (7→5) and **recovered 2 of the 3 south omissions** (ev29, ev34, both
  `sur→este`). **ev22 (`sur→norte`) is unrecoverable** here (no reconnectable fragment
  chain). It did not "fix 6/7 ghosts": the conservative criterion leaves the doubtful unresolved, and
  several chains only consolidate short fragments into already-matched crossings (without changing the
  count).
- **West defect (precision 0.938→0.889):** it is not over-merging (`false_merges=0`). It is `[24,32]`
  (short west fragment `tid24` 16 f + partial `core→norte` `tid32` 770 f) = a **real west→north
  (probably ev3)** whose reconstructed entry crossing falls at t≈12.9 s vs GT ev3 11.3 s → ~1.6 s,
  outside the 1.0 s window → ghost + ev3 omission. It is a timing failure in a real recovery, not an
  incorrect merge. Accepted: the net is positive and tightening further risks losing south recoveries
  (they share the buffer window).

## Limits

- **Temporal uncertainty of reconnected fragments:** the reconnected events have more
  uncertainty in the stop-line crossing instant than whole tracks, because the crossing
  is reconstructed from pieces (for example the west defect: crossing 1.6 s off the GT; and
  the south `acierto_salida` 0.917→0.857). It is a limitation of the temporal/spatial precision of
  the recovered fragments.
- **It does not suppress co-temporal duplicates** (tid70 = two ids over the same bus overlapping in
  time): a different mechanism from gap-based reconnection (which requires `gap>0`). A box-overlap
  suppressor is not built, because of the data: tid70 is not of sustained high IoU (IoU median
  0.14, IoS 0.45, ¼ area; the 0.99 was a one-off maximum of the *handoff*), and box overlap does
  not separate a duplicate from distinct vehicles (e.g. (44,115), both cross-arm matching distinct
  GTs, IoU 0.99). The co-temporal duplicate belongs to the count metric, not the route one (it inflates
  the count; in route it is a single event, tid70), and it is handled in that metric, not here. tid70
  stands as a known limit: a co-temporal duplication artifact in large vehicles, a south ghost
  that does not harm the route.
- **It does not resolve lane** (the event layer gives a coarse approach; the lane corresponds to the
  zones layer).
- **Script constants:** the angle threshold (60°) and `HEADING_FRAMES` (5) are documented script
  constants; the buffer and the corridor do come from config.

## References

- `docs/modules/tracking.md`: origin of the tracks CSV; the appearance re-id not used
  here.
- `docs/modules/track_events.md` and `docs/modules/matching_sombras.md`: the evaluation pipeline
  re-run over the merged tracks.
- `docs/modules/causal_reconnect.md`: the causal (online) version of this reconnection.
- Site config: `config/sites/rusia/site.yaml`, `config/sites/rusia/zones.yaml`,
  `config/sites/rusia/turns.csv`.

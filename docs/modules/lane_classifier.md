# Module — Lane classifier (entry and exit, bands in the image plane)

Adds `entry_lane` and `exit_lane` to the per-track event: the vehicle's **entry lane** and **exit
lane**, in the **image plane**, measured **right at the stop line**, over **reconnected** tracks. The
convention is a fixed fact: lane 1 = the vehicle's right in its direction of travel; entry and exit are
measured in opposite frames. It is integrated in `scripts/derive_track_events.py`; the calibration
lives in `scripts/calibrate_lane_bands.py` and the validation in `scripts/measure_lane_accuracy.py`.

The criteria are measured (separability analysis), not set by eye:

- **Image plane, not homography:** where the lane separates, it already separates in the image; in the
  far field the homography amplifies the noise.
- **Measured at the stop line, not by averaging:** near the stop the vehicles are ordered; the far
  field hides them. Also verified for **exit**: it separates better at the egress crossing than farther
  out (west LOO 0 %→18-20 % as it moves away; east d′ drops). The premise transfers.
- **Entry and exit share the plane (`stop_lines`) and the transverse axis `u`;** they differ only in
  the **direction of the crossing** (entry = first crossing −→+; exit = last +→−, the same plane
  crossed outward) and in the **centroids**. No new egress line is needed (there are no egress control
  points, and none are needed), no clicks, no homography.

## Classification

1. **Transverse coordinate `s`** (`lane_cross_coord`, ±3 frame window): the ground contact at the stop
   crossing is projected onto the **direction of the stop-line segment**.
2. **Nearest centroid** (`classify_lane`): the label whose centroid `s` (from `lane_bands` in
   `zones.yaml`) is closest. Equivalent to the LOO-validated classifier.
3. **`''` (unresolved)** if there is no clean crossing, the approach is `arm_only`, or the point falls
   in the core.

## Calibrated bands (Rusia) and per-approach resolution rule

**Replicable rule:** an **individual** lane where its classification error is of the order of the
approach-layer error (~5–16 %) **and** there is sample to support it; **grouped** if the overlap is
physical; **arm_only** if the sample does not support it.

`config/sites/rusia/zones.yaml` → `lane_bands.entry` and `lane_bands.exit` (`centroids_s` in px, with
provenance). Calibrated from GT v2 over reconnected tracks:

| | approach | resolution | `centroids_s` (px) | N | reason |
|---|---|---|---|---|---|
| **entry** | east | per_lane (1/2/3) | 202.2 / 156.9 / 111.1 | 3/13/4 | separable |
| | north | per_lane (1/2/3) | 189.9 / 147.2 / 101.9 | 13/4/2 | separable (lane 3 N=2) |
| | west | grouped (1·{2,3}) | 1:164.6 · 2-3:117.5 | 1:2 · {2,3}:14 | pair 2-3 = **sensor** limit |
| | south | arm_only | — | 4/3/3 | **sample** limit |
| **exit** | east | per_lane (1/2) | 45.6 / 74.0 | 13/9 | ~14 % **behavioral** error |
| | west | per_lane (1/2) | 20.6 / 39.3 | 11/6 | separable (LOO 0/17) |
| | north | per_lane (1) | 21.8 | 8 | 1 egress lane (trivial) |
| | south | per_lane (1) | 24.0 | 15 | 1 egress lane (trivial) |

## Input and output

- **Input:** reconnected tracks, `zones.yaml` (`stop_lines` + `lane_bands.entry/exit`),
  `site.yaml` (image center).
- **Output:** the `entry_lane` and `exit_lane` columns in `track_events.csv`. Values: `"1"/"2"/"3"`
  (per_lane in all approaches), `""` (unresolved: partials or no clean crossing). *Type note:*
  the round-trip to CSV can read numeric-only columns as `1.0`; the consumers normalize
  (`measure_lane_accuracy.norm_lane`).

## Measured validation (conditional on the approach, matched pairs)

**Lane (deployed in sample | LOO out of sample):**

| | approach | deployed accuracy | out-of-sample LOO |
|---|---|---|---|
| entry | east | 20/20 = 100 % | 0 % err |
| | north | 18/19 = 95 % | 5 % err |
| | west (per_lane) | 14/16 = 88 % | 12 % err (declared 2\|3 overlap: bus drift) |
| | south (per_lane) | 11/11 | 0 % err — **sparse sample n=4/3/4, not validated** |
| exit | east | 20/24 = 83 % | **14 % err** (behavioral) |
| | west | 19/19 = 100 % | 0 % err |
| | north/south | trivial (1 lane) | — |

Deployed and LOO agree (they generalize). **South→west anchor** (left turn → west egress **lane 1**, GT
`exit_lane=1`): the 4 matched give **shadow west/1** (ev49 is a miss). The west exit band is `{1:24.3,
2:66.5}`; a left turn from the south egresses through lane 1, not lane 2 (SUMO draws lane 1 by the
canonical assignment of the connection with no origin lane).

**Route levels: approach → + entry lane → full route (4 fields):**

| approach | approach (recall) | + entry lane | full route |
|---|---|---|---|
| east | 91 % | 91 % | 86 % |
| north | 100 % | 95 % | 95 % |
| west | 94 % | 82 % | 82 % |
| south | 87 % | 73 % | 53 % |
| **total** | **93 %** | **86 %** | **81 %** |
| **total without south** | 95 % | **90 %** | **88 %** |

The cost of requiring the **full route** (adding approach and exit lane) over + entry lane is ~12 pts
in the approaches with a lane (91 %→79 %).

## Three types of limit

There are three real and distinct causes of not resolving lane, none of them a defect:

- **Sensor (west entry, pair 2-3):** the camera separates that pair with **overlap** (bus drift; a
  28.1 px gap with sd ~11, LOO 14/16). It is an **observation** limit, declared. per_lane is used
  despite the overlap: grouping would maximize aggregate accuracy but would degrade the **spatial**
  fidelity of the drawing; the classification cost is reported, not hidden.
- **Behavioral (east exit):** the camera **does separate** (~14 % error), but the error is
  **behavioral**: drivers use any egress lane in the east (GT: left turn 4/2, right 4/1, straight 7/6).
  A **faithful shadow** must **reflect** that human dispersion, not correct it by grouping; replicating
  variable behavior is exactly what a shadow does. Hence it stays individual by the rule (14 % within
  the approach error band).
- **Sample (south entry):** at the stop it separates cleanly (gaps ~34 px, sd ≤ 4.3, LOO 11/11) but
  with n=4/3/4. **It is not that "the south does not separate"**; it is lack of data. It stays per_lane
  with the limitation explicit in `zones.yaml`: it is not reportable as validated classification until
  more sample is available (long video + south lane GT).

Sensor, behavioral, and sample are three distinct causes, and only the sample one is reduced with more
data. Distinguishing them is a finding of the work, not a defect.

## Limits

- It does not re-derive the approach-only baseline (preserved as a reference); the lane lives in the
  reconnected state.
- **South entry:** sample limit (more data could resolve it). **West entry {2,3}:** physical limit
  (stays grouped). **East exit:** behavioral limit (stays individual, reflected).
- Small N in general (north lane 3 N=2; several cells 2–6): directional accuracies, wide CIs.

## References

- `docs/modules/track_events.md`: the event layer that gains `entry_lane` and `exit_lane`.
- `docs/modules/zones.md`: the zones layer where `stop_lines` and `lane_bands` live.
- `docs/modules/class_stabilizer.md`: class stabilization in the same event layer.
- `docs/modules/route.md`: the route piece that uses the entry lane to predict the turn.
- Site config: `config/sites/rusia/zones.yaml`, `config/sites/rusia/site.yaml`.

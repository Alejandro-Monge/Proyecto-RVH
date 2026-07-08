# Module — event-per-track layer (`scripts/derive_track_events.py`)

Converts each **track** of the tracking CSV into an **event**: **entry** and **exit** approach and
time. It produces the unit on which the shadow-GT matching operates, but it does not match against
the GT (that corresponds to the matching stage, `docs/modules/matching_sombras.md`).

The approach is classified **coarsely** by the **image corner** where the track appears
(entry) and disappears (exit). It does not require the fine zones of the zones layer: a coarse
decision per corner is enough for the matching, which anchors by **entry approach + time**.

## Input and output

**Input:**
- `verify_out/tracks_rusia.csv` — tracking output (`track_id, frame_index, timestamp_s,
  x1,y1,x2,y2, cx,cy, class_id, confidence`).
- `config/sites/<id>/site.yaml` — `frame_width`/`frame_height` (image center) and `arm_set`.
- `config/sites/<id>/zones.yaml` — **exclusion** polygon(s) (west parking lot) and
  **stop-line segments** per approach (`stop_lines`), for the crossing instant.
- `verify_out/homography_gcp/control_points.csv` — **only** for the cross-check (anchor
  calibrated per approach); it is not the source of the production classifier.

**Output:** `verify_out/events/<id>/track_events.csv`, **one event per NON-excluded track**.
Columns:

| Column | Meaning |
|---|---|
| `track_id`, `n_obs` | track id and number of observations |
| `entry_frame`, `exit_frame` | first / last frame of the track |
| `first_seen_s`, `last_seen_s` | time (s) of the track's **appearance** (first/last observation) |
| `entry_time_s`, `exit_time_s` | time (s) of the **stop-line crossing** (physical instant = the one the GT measures) |
| `entry_time_fallback`, `exit_time_fallback` | `True` if there was no crossing and it fell back to the appearance |
| `entry_gx, entry_gy, exit_gx, exit_gy` | entry / exit ground contact (px) |
| `entry_dist_center_px`, `exit_dist_center_px` | distance to the image center (px) |
| `entry_arm`, `exit_arm` | approach `{norte,sur,este,oeste}` or `core` if it falls in the core |
| `status` | `complete` (both corners clear) or `partial` |
| `movement` | `cross` (entry≠exit) · `same` (same approach) · empty if partial |

**Excluded** tracks (parking lot) are not written (no events are generated for parked vehicles);
they are listed in the console summary.

### Instants: appearance ≠ physical entry

The **approach** is classified by the **appearance** corner (validated, see "Corner criterion"),
but the entry/exit **time** is not the appearance: it is the **stop-line crossing** of the
approach, the same physical event the human GT marks. The appearance occurs **20-40 s earlier** in
far approaches (the vehicle is seen from afar long before it crosses, and it may wait at
red); using it as the entry time makes shadow and GT incomparable. The appearance is kept
as `first_seen_s`/`last_seen_s` (a real fact of the track, useful for delay/fragmentation).

**How the crossing is computed:** signed distance from the ground contact to the **stop-line
segment** of the approach (`stop_lines` in `zones.yaml`, calibrated endpoints from `control_points.csv`),
oriented toward the image center (+ = intersection side). **Entry** = first crossing −→+;
**exit** = last +→−; sub-frame interpolated. If the track does not cross (it appears already past the
line, or the approach is `core`), it falls back to the appearance and is marked `*_fallback`.

## Reference point

The **ground contact** `(gx, gy) = ((x1+x2)/2, y2)` via
`digital_shadow/geometry/ground_contact.py` (`docs/modules/ground_contact.md`). Not the centroid
(it introduces height parallax).

## Corner criterion (coarse)

1. **Entry point** = median of the ground contact in the first `WINDOW_FRAMES` frames;
   **exit point** = median in the last `WINDOW_FRAMES`. The median absorbs detection jitter
   without averaging over real movement: at ~29.6 fps, 5 frames ≈ 0.17 s, in which a
   vehicle in the periphery moves <30 px, well below the ~300 px scale that separates
   one corner from another, so the classification is insensitive to `WINDOW` for peripheral
   points (verified, see "Measured validation").
2. **Approach = quadrant of the point relative to the image center** (`W/2, H/2`), per the corner
   mapping of the site: top-left = south, top-right = west, bottom-left = east, bottom-right = north. The
   image center was chosen and not the calibrated J5 core because for the peripheral points —those of
   clean events— both splits give the same result (verified: 79 identical cross-arm
   events; the only tracks that differ are stuck to the center and are partial
   anyway), and the image center does not introduce calibration constants into the logic (`W,H` already
   live in `site.yaml`). The calibrated per-approach anchors (centroid of the homography
   control points) are used only as a check: they agree with the quadrant in **98.9 % (entry) /
   98.4 % (exit)**.
3. **Partial** if the entry or the exit falls to `≤ CORE_RADIUS_PX` of the center: the track is born or
   dies inside the intersection (fragment by occlusion or ghost), with no clear approach mouth. It is not
   corrected here (reconnection is a separate front); it is marked and counted.

## Parameters

| Parameter | Value | Justification |
|---|---|---|
| `WINDOW_FRAMES` | 5 | ~0.17 s; insensitive for periphery |
| `CORE_RADIUS_PX` | 120 | < every stop line (~249–442 px from the center) → no approach mouth is called `core`; inside the J5 box (CB vertices ~115–258 px) → isolates the center. Stable count in R∈[80,150] |
| `EXCLUSION_FRACTION` | 0.5 | track with >50 % of ground contact in exclusion = parked |

These are documented constants of the script, with their provenance. The corner→cardinal mapping is a
datum of the site; the stop-line segments already live in config (`zones.yaml: stop_lines`).

## Measured validation (approaches and times)

Over `verify_out/tracks_rusia.csv` (194 tracks):

- **Exclusion:** discards exactly `{12, 36, 116, 128}` = the 4 parking-lot tracks
  documented in `zones.yaml` (100 % inside). **190 events** written.
- **Taxonomy:** 180 complete (**79 cross-arm**, 101 same-arm) + **10 partial**.
- **Shadow O-D (cross-arm) vs GT v2 (73):** the origin-destination matrix reproduces that of the GT almost
  cell by cell; total 79 vs 73 (+6, over-count expected from fragmentation).
- **Known cases — approach:** the 4 `sur→oeste` turns (tid 99/68/88/92, the ones that anchored the
  correction of the 180° homography) come out `sur→oeste`, complete/cross.
- **Known cases — time (stop-line crossing):** the `entry_time_s` of those 4 turns checks out
  with the GT to **|diff| ≤ 0.19 s** (tid99 98.0 vs 98.1; tid68 96.3 vs 96.4; tid88 93.1 vs 93.3;
  tid92 87.2 vs 87.3), versus the **20-40 s** error the appearance gave. Globally, the
  crossing is at **median 0.24 s** (p90 0.65 s) from the nearest same-arm GT; only **2 cross-arm at
  >3 s** (both in the south). Fallbacks (no crossing): 10/79 entry, 9/79 exit → fall back to the
  appearance.
- **Independent corroboration (geometry):** the **south** `same-arm` tracks (n=32) have
  median displacement **9 px**, median `n_obs` **224** and median duration **9.5 s** = traffic
  stopped at red (they are not U-turns or fragments). The east/north/west `same-arm` are short
  (0.5–1.7 s) = brief approaches / ghosts.
- **Sensitivity:** cross-arm ∈ [73, 83] in `N∈{1,3,5,10} × R∈{80,120,150}` (stable zone); the chosen
  point (N=5, R=120) gives 79.

The `same-arm` (101) and the partials (10) are not noise: they are the fragmentation and the
non-through traffic visible at this level. Their count is informative (how much fragmentation is seen
before reconnecting). The matching stage decides which ones enter the matching (which anchors by entry).

## Limits

- **Does not match** against the GT (matching stage) nor computes metrics.
- **Does not reconnect** fragments (separate front): it marks them as `partial`.
- **Does not resolve lane** (that needs the fine zones of the zones layer): only coarse approach by
  corner.
- **Does not apply the exclusion in the pipeline** (which would go after the detector, before the tracking):
  here it operates on the already-tracked CSV, filtering complete tracks.

## References

- `docs/modules/tracking.md`: source of the tracks CSV that this layer consumes.
- `docs/modules/ground_contact.md`: the ground contact point used as reference.
- `docs/modules/matching_sombras.md`: the matching stage that consumes these events.
- `docs/modules/zones.md`: the zones layer (exclusion and fine lane).
- Site config: `config/sites/rusia/site.yaml`, `config/sites/rusia/zones.yaml`.

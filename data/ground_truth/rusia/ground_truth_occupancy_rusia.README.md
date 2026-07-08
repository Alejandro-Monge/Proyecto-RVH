# Approach OCCUPANCY GT — annotation guidelines

File to fill in: `ground_truth_occupancy_rusia.csv` (template in this directory).
Support video: `verify_out/inspeccion_approach/gt_occupancy_labeling_tramo.mp4` —
it has the 12 lane-entry detectors burned in (lines by color: c1 green,
c2 orange, c3 blue), the clock in absolute seconds with milliseconds (top-right) and the
contact point of each vehicle's TRACKING (system context; it is NOT the
annotation reference — see the NOSE rule below).

It is the SIBLING file of the crossings GT (`ground_truth_rusia_v2_carril_processed.csv`):
it complements it, it does NOT replace it or repeat it.

## Time base

Absolute seconds with **t = 0 at the start of the 180 s segment** — the SAME base as the
crossings GT and as the `NN.n s` clock burned into the video. It is annotated with one decimal
(video resolution ≈ 0.03 s/frame; one decimal is enough and honest).

## ⚠ `zone_enter_time_s` is NOT `entry_time_s`

- `zone_enter_time_s` (THIS file) = the instant the vehicle **enters the approach
  zone** (crosses its lane's entry detector, at the back of the queue).
- `entry_time_s` (crossings GT) = the instant it **crosses the stop line** (leaves
  the queue toward the intersection).
These are two distinct physical events, separated by the entire wait in the queue. To avoid
confusing instant names, the field has its own name and is NEVER called
`entry_time_s`.

## What is annotated (NOSE rule — reference criterion)

The time is marked when the **NOSE of the vehicle (its physical front)** crosses the
detector line, as observed by the annotator. **NOT the white dot in the video**: that
circle is the contact point of the TRACKING (the system's reference, not the annotator's)
and is left visible only as context for what the system sees.

Why the nose and not the contact point: in the south/west, at far field, the tracking
contact point is unreliable (it appears late or is lost). The nose lets the
annotator record the **real physical event** even where the system would fail — which
is exactly the purpose of the GT. The criterion is **UNIFORM across the four arms**
(references are not mixed by arm). The systematic nose↔ground-contact offset
is bounded by the vehicle's projected length and is documented in
`docs/modules/occupancy_zones.md`.

## Column `cruce_pare_time_s_ref` — REFERENCE ONLY (not annotated, not measured)

Pre-populated by mechanical derivation: it is the `entry_time_s` of the crossings GT (stop-line
crossing) for the same `event_id`. It serves only to orient the filling in — to know in
what temporal order to look for each vehicle in the video. **It is not annotated, the counting
system does not read it, it does not enter any metric.**

- A different name from `entry_time_s` and from `zone_enter_time_s` ON PURPOSE: it is the
  STOP-line crossing, which occurs AFTER the zone entry. For a single vehicle:
  **`zone_enter_time_s` < `cruce_pare_time_s_ref`** (the nose crosses the zone
  detector before reaching the stop line). If while annotating it comes out the other way, something is wrong
  (wrong vehicle or wrong detector).
- Crossers (1-73): populated. Non-crossers (74-80): empty (they do not cross the stop line).
- The ~11 entry-censored ones: the reference IS present (they cross the stop line afterward),
  even though their `zone_enter_time_s` stays empty due to censoring.

## Crossers (the 73 pre-populated ones, `crosses=si`)

- They already carry `event_id` (the SAME as the crossings GT), `entry_arm`, `entry_lane` and
  `vehicle_class` copied from that GT. Only **`zone_enter_time_s`** is annotated.
- **`zone_exit_time_s` IS LEFT EMPTY**: their zone exit is the `entry_time_s` of the crossings
  GT (the stop-line crossing); it is not re-annotated.
- **The pre-populated `entry_lane` comes from the crossings GT = the lane observed AT THE STOP-LINE
  CROSSING.** If, when observing the ZONE ENTRY, the annotator sees a different lane —
  an expected case at the back of south c2, where the lane does not separate —, **what is
  observed at the entry prevails**: correct `entry_lane` to what is seen and **annotate the
  discrepancy in `notes`** (e.g. `pare=c2, zona=sin_carril`).

## Non-crossers (NEW rows, by hand)

Vehicles that enter an approach zone but do NOT cross the stop line within the clip
(they stay in the queue at the end, or back up/park):
- `event_id`: NEW running number from **74** (do not reuse ids from the crossings GT).
- `crosses=no`; `entry_arm`, `entry_lane`, `vehicle_class` observed.
- `zone_enter_time_s`: as above. `zone_exit_time_s`: when it stops being present
  in the zone (or the end of the clip, 180.0, if it is still there).

## The `na` value in times: "never crosses the lane detector"

`zone_enter_time_s = na` and `zone_exit_time_s = na` (literal) mark the vehicle that
vision DOES see inside the ARM polygon but that **never crosses its lane's detector**
within the clip (e.g.: ev77-79, the south queue behind the tiny S-c2
detector). Three distinct states, not to be confused:
- annotated time → normal lane zone interval;
- EMPTY → depending on `entrada_censurada`: `si` = censoring (final datum), `no` = not yet annotated;
- `na` → never entered a lane zone (FINAL datum): it gets `entry_lane = sin_carril`,
  counts in the arm AGGREGATE and stays **outside per-event matching** (no temporal
  anchor — no vehicle is annotated by arm detector, so it would be inconsistent to
  anchor only these ones).

## The `sin_carril` rule (fallback FORBIDDEN — including the discard one)

A vehicle that never separates lane (the back of south c2 and similar) is annotated as an
entry to the ARM with `entry_lane = sin_carril` (literal, not empty: empty means
"not yet annotated"). **Forcing it into a neighboring lane is FORBIDDEN** — it would
contaminate that lane's metric. The detector it crosses in that case is conceptually the edge
of the ARM polygon; the time is annotated all the same.

**No lane by DISCARD either**: the rule "whatever is not c1 or c3 is c2" (and similar)
is a rule of the SYSTEM (its canonical fallback), NOT of the GT. In the GT there are only two
valid outputs: the OBSERVED lane or `sin_carril`. Never a lane deduced by
elimination.

## Censoring at BOTH edges of the clip (column `entrada_censurada`)

The clip cuts reality at t=0 and t=180: **the unobserved event is not invented**.

- **Censored entry** (`entrada_censurada = si`): a vehicle ALREADY present inside a
  lane at the start of the clip — its detector crossing occurred before t=0 and was not
  observed. `zone_enter_time_s` is left EMPTY and it occupies the zone from t=0. There are ~11 like this
  among the crossers (the annotator marks them upon seeing them already inside in the first
  frame). `crosses=si` is kept: they cross the stop line afterward, and their zone exit is already
  in the crossings GT.
- **Censored exit** (non-crossers): `zone_exit_time_s` EMPTY = it was still in the zone at the
  cut of the clip (t=180). If it is observed to leave earlier, the time is annotated.
- **Critical distinction** (this is why `entrada_censurada` must ALWAYS be filled in for
  crossers): empty `zone_enter_time_s` + `entrada_censurada=si` → present from the
  start (FINAL datum); empty + `entrada_censurada=no` → datum NOT YET ANNOTATED
  (pending). They cannot be confused.
- The template carries `entrada_censurada=no` in the 73 crossers by default: when
  annotating, change to `si` the ~11 already present at the start. In rows 74-80 the
  column comes empty and is filled by hand with the rest of the row.

## Warning: `S-c2` detector

The lane-2 detector of the south is **tiny** (17 px deep, ~1 vehicle
position — minimal coverage declared on purpose). It will be hard to see: annotate
that zone carefully and with zoom. When in doubt between c2 and a neighbor in the deep south:
`sin_carril` + note, never guess.

## Valid values (so the CSV parses without cleanup)

`crosses` ∈ {`si`, `no`} (no accent) · `entry_arm` ∈ {`norte`, `este`, `sur`,
`oeste`} · `entry_lane` ∈ {`1`, `2`, `3`, `sin_carril`} · `vehicle_class` ∈ {`car`,
`bus`, `truck`} · `entrada_censurada` ∈ {`si`, `no`} · times in seconds with a decimal
point or `na` (never crosses the lane detector; see rule above) · `notes` free text.

# Module — `digital_shadow/occupancy/`: approach zones and instantaneous count

A layer independent of the causal pipeline. It loads the occupancy geometry
(`config/sites/<site>/approach_zones.yaml`) and counts how many vehicles are inside
each zone in the present frame.

## Outputs

`occupancy_counts` produces the **raw instantaneous count per frame**: how many
ground-contact points fall inside each polygon in frame t. **No track_id, no
time, no state.** **It is not the final measured occupancy**: the dwell threshold
and the comparison against the occupancy ground truth are a **later layer** that does
require temporal identity (following the same vehicle across frames). This function is
the base level, **not the number that goes to the paper**. Any reportable occupancy
figure comes from the upper layer, with its own validation against GT. The
instrument of that validation (tuning θ/g against the occupancy GT, in-sample, not for
the paper) is `scripts/measure_occupancy_accuracy.py` →
`verify_out/occupancy_accuracy/`.

## Purpose

The per-approach count is robust down to the tail of the queue; the per-lane count is only
valid as far as that lane separates visually (depth declared per lane). The
geometry lives in `approach_zones.yaml`, apart from `zones.yaml`, because
`zones.yaml` is pinned by md5 in the guard of the causal command and of the figures:
this layer iterates on its own life cycle without breaking those pins.

## Input and output

### `load_approach_zones(site_id, path=None) -> ApproachZones`

- Reads `config/sites/<site_id>/approach_zones.yaml` (`path` only for tests).
- **Fail-fast**: aborts with `ValueError` (listing all failures, with the offending
  polygon named) if something does not add up: <4 vertices; `entry_side` out of range or
  **inverted** with respect to the approach's stop line (the entry edge must
  be farther from the stop than the centroid — it is blocking: if inverted it would ruin
  the entry times of the occupancy GT); lane ⊄ approach; overlapping lanes;
  overlapping approaches; dims ≠ `site.yaml`; `coverage` ∉ {full, partial}.
- Returns `ApproachZones(site_id, frame_width, frame_height, arms)` with
  `arms: {brazo: ArmZone(polygon, entry_side, lanes: {k: LaneZone(polygon,
  entry_side, coverage, coverage_note)})}`. Polygons in image px at full
  resolution, implicit closure, membership by ground contact (the same contract as
  `zones.yaml`).

### `classify_points(points, zones, exclusion_polygons=()) -> (arm_col, lane_col)`

**The single source of point→zone membership** (see the
"Single source" section). Returns two arrays aligned with `points`: `arm_col`
(approach, or `""` = excluded/outside) and `lane_col` (lane k, or `-1` = `sin_carril`
within the approach; filter by `arm_col` first). A single assignment per point,
well defined because the loader guarantees disjoint approaches and lanes ⊂ approach
(fail-fast); no fallback to a neighboring lane. Consumers: `occupancy_counts`
(aggregation), waiting boxes (labels per track), the metric instrument
(membership per row), and the 3D emitter.

### `occupancy_counts(points, zones, exclusion_polygons=()) -> dict`

It is the **aggregation** of `classify_points` (a single
membership logic underneath). External contract:

- `points`: ground contacts `(gx, gy)` of frame t (iterable of pairs or Nx2 array).
  The caller applies the same ground-contact reference as the pipeline
  (`ground_contact.py`: `((x1+x2)/2, y2)`).
- `exclusion_polygons`: discarded before counting (for example, the parking lot from
  `zones.yaml`, the same exclusion the pipeline applies).
- Returns `{brazo: {"total": int, "lanes": {k: int}, "sin_carril": int}}`.
  - `total`: inside the approach polygon (reliable to the end).
  - `lanes[k]`: inside the polygon of lane k (coverage declared in config;
    in west c2/c3 it drags the declared sensor overlap; in south c2 the coverage is
    minimal ~1 position).
  - `sin_carril`: inside the approach, outside all lanes (non-separable end).
    **There is no fallback to a neighboring lane** — the residual is an explicit channel.
  - Invariant: `total == sum(lanes) + sin_carril`.

## Causality and non-interference

- A pure function of the present frame: it satisfies I1/I3 of the causal contract
  (`digital_shadow/causal/contract.py`); verified in tests with `check_pure_lookup`
  (determinism, idempotency, order independence).
- No causal production module imports this package; this package does not write
  to any `Sink`, does not create chains, and does not touch the pinned inputs of the
  causal command. Verified: causal probe PASS (70 segment / 104
  long) with byte-identical readouts.

## GT reference vs system reference

The **occupancy GT marks the entry by the vehicle's front** (its physical front
crossing the detector, observed by the human annotator), whereas the **system
measures membership by the ground contact** (`((x1+x2)/2, y2)` of the box). Reason for the
criterion: in south/west in the far field the tracking contact point is not very
reliable (late appearance / loss); the front lets the annotator record the real
physical event right where the system would fail — which is the function of the GT. The criterion
is **uniform across the four approaches** (references are not mixed per approach).

The front↔ground-contact offset is **systematic and bounded by the vehicle's projected
length** in the image. For the **aggregate per-frame occupancy count it is
negligible**: the edges of the presence interval shift equally at the entry
and at the exit of the zone, so they cancel in the instantaneous count
(the vehicle contributes the same number of frames inside the zone, shifted by ~one
vehicle length). Where it does matter is in the per-event comparison of entry times
(future validation layer): there the offset must be treated as a known systematic
bias, not as a system error.

### Censoring at the clip edges (occupancy GT)

The occupancy GT declares the censoring at both edges instead of inventing unobserved
events: **censored entry** (`entrada_censurada=si`) = a vehicle already inside the
zone at t=0, detector crossing not observed, empty `zone_enter_time_s`, occupies from
the start of the clip; **censored exit** (non-crossers with empty `zone_exit_time_s`)
= still in the zone at the cut (t=180). The `entrada_censurada` column disambiguates
the empty value: `si` → final datum (present from the start); `no` → datum not yet
annotated. The future validation layer must treat the censored intervals as such
(presence known only within [0, t] or [t, 180]), not as entries/exits at
the exact edge.

## Parameters

The instantaneous count (`occupancy_counts`) still has no thresholds of its own: the geometry
(polygons, coverages) comes from config, traced over the GT
approaches (the "queue by speed" variant is still a
future refinement with sensitivity analysis).

The measurement layer does have an operating point, versioned
in `approach_zones.yaml: measurement` (config, not hardcode):
**θ = 0.75 s** (minimum accumulated dwell to count) and **g = 0.5 s** (merging of
presence gaps of the same track). Calibrated by a joint θ×g sweep against the
count series of the occupancy GT (`scripts/measure_occupancy_accuracy.py --sweep`);
chosen within the stable MAE zone by the event tradeoff (fragments
149→20, ghosts 17→1, at the cost of 4 short free-flow transits of the south that are
not queue), prioritizing occupancy fidelity (pent-up demand) over ephemeral
transits. Contract causality: θ is a condition on past/present (accumulated dwell
up to t) and g merges when the second fragment has already appeared — neither reads
the future; both are bounded latency, like the reconstructor's R2 removal. The causal probe
re-verified PASS after fixing the point (the layer still does not touch the causal pipeline).

**Declared limits (not corrected by threshold):** (1) **in-sample** calibration over
the single 180 s clip — it does not migrate to the paper as validation; (2) a structural
floor ~0.10 veh of MAE with a deficit localized in **south/c3** (queue censored in the far
field: bias −0.54, the tracking loses the deep queue) = sensor limit; (3) measured
reference offsets: front→contact ≈ 0 (±0.2 s) in east/north/west and −0.7 s with
large variance only in south; Δexit (+0.3–0.4 s only in frontal approaches) is contact-vs-front
geometry, consistent with the crossing GT having anchored at the front (without an
anomalous bias).

## Tests

`tests/test_occupancy_zones.py` (11): loader over the promoted real config
(coverages included), 4 fail-fast aborts with synthetic fixtures anchored to the
real south stop, count (lane/total/residual, no-fallback, exclusion, empty
frame), structural invariant over the real config with random points, and
`check_pure_lookup` over `occupancy_counts`.

## Presentation layer: waiting boxes (`--waiting-boxes`, default OFF)

`digital_shadow/occupancy/waiting_boxes.py` (pure logic) + `WaitingBoxLayer` (TraCI glue
in the causal demo): draws as translucent polygons (alpha 120, exact color and dimension
of the real vTypes read at runtime — `vehicletype.getLength/getWidth/getColor`) the
vehicles waiting in the approach before the stop. Presence and class from the detector, raw
homographic position of the present frame, geometric orientation of the lane (facing the stop;
`lane.getShape`), never route. Causal: it appears after θ of accumulated dwell and survives
gaps ≤ g (the `measurement` operating point); the class is the vote of the past. It is extinguished at
the stop crossing (the causal vehicle is born), by an exhausted gap, or by end of track. Box overlap
in a dense queue is drawn as is (by design): it is the real uncertainty
of the far field. It does not touch the measurement: verified, byte-identical segment readout to the
safe point and causal probe PASS with the per-class width (VTYPE_DEF declares
width: car 1.8 / bus 2.5 / truck 2.4 = SUMO defaults).

## Single source of membership

The concept "who is in a zone" has a single path: `classify_points`, over the
`_inside` primitive and `approach_zones.yaml`. The three users consume it without
re-deriving the count logic: (1) `occupancy_counts`, (2) the own
classification of `WaitingBoxLayer` (causal harness), (3) the masks of
`measure_occupancy_accuracy`. Reason (the c2-south doctrine, applied to the whole layer):
two paths that give the same today can diverge; a single source prevents it.
Equivalence demonstrated over the reference segment (92 758 tracking rows,
5324 frames): **0 divergences in 106 480 cells**
(frame × approach × {total, c1, c2, c3, sin_carril}); the labels and the masks
coincide by path (0/92 758; 0 in 16 zones × 85 094 rows).

- **No latent divergences**: the assignment is unique and uniform, correct by
  construction. The instrument's lane mask intersects with the approach
  (lane ⊂ approach), so an overlap cannot make one path count
  double and another apply "last wins".
- **Instrument determinism** (the same set→output pattern as the
  messages `validation_log`): `tier_a_zonas.csv` orders its rows
  contractually by `zona` (`sorted()` in the iteration), reproducible byte for
  byte without PYTHONHASHSEED. **Reference**:
  `tier_a_zonas.csv = 97dd359d7ab05665611407d18fd50545`
  (sorted content `6d46b09a…`). The 3 remaining set iterations of the
  harness (`:508/:539/:544`) do not touch file/summary/log — inventoried.
- **Doctrine**: every verification that re-generates a reference artifact
  **backs up the artifact before running**.

## Measurement vs presentation

**What it is**: perception of approach occupancy (instantaneous count by zones) + a
presentation layer over the demo (waiting boxes). **What it is not**: the boxes are not
measurement — nothing that is drawn feeds a metric.

### The line between measurement and presentation

| piece | nature | validates against | where |
|---|---|---|---|
| instantaneous count (`occupancy_counts`) | measurement (base) | — (pure, tested) | 4 approaches, 12 zones |
| measured layer (θ=0.75/g=0.5 from `measurement`) | measurement (tuning) | occupancy GT, segment only (declared in-sample) | `measure_occupancy_accuracy` instrument |
| waiting boxes (`--waiting-boxes`, ON by default in visual modes) | presentation | visual validation (not metric) | causal demo; never in `--readout` |
| box lane anchoring | presentation | — (reuses the audited position.py doctrine) | 4 approaches |
| c2-south rule (south far end → c2 axis if c1 empty) | presentation | — (`occupancy_counts` literal as condition) | south approach only |

The south far-end vehicle is `sin_carril` in the metric and in the GT even though its box is
drawn anchored to c2. The long video has no occupancy GT: any inspection there is
visual, never a measurement.

### Declared limits (the ones that go to the paper)

1. **In-sample θ/g calibration** over the single 180 s clip — it does not migrate as validation.
2. **Structural floor ~0.10 veh of MAE** with a deficit localized in **south/c3** (queue
   censored in the far field; bias −0.54) = sensor limit.
3. **West longitudinal**: error 4.52 m (homography horizon limit,
   measured); the west boxes drag it — to visual validation; the lateral does not (the
   anchoring fixes it to the lane).
4. **Box overlap in a dense queue is drawn as is** (by design): it is the
   real perception uncertainty, it is not de-overlapped.
5. Measured reference offsets: front→contact ≈0 (±0.2 s) except south (−0.7 s, high
   variance); Δexit +0.3–0.4 s only frontal approaches = geometry, crossing GT without bias.

## References

- `docs/modules/zones.md`: the image-plane polygon contract that this layer shares.
- `docs/modules/ground_contact.md`: the ground-contact point used for membership.
- `docs/modules/causal_contract.md`: the causal contract (I1/I3) that `occupancy_counts` satisfies.
- `docs/modules/presentation.md`: the presentation smoothing of the causal demo.
- Site config: `config/sites/rusia/approach_zones.yaml`, `config/sites/rusia/zones.yaml`.
- `scripts/measure_occupancy_accuracy.py`, `tests/test_occupancy_zones.py`.

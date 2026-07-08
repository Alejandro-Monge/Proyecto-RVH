# Module — Ground truth of the messages shadow and measurement protocol (`data/ground_truth/rusia/gt_mensajes_sim_*`)

Reference truth of the V2I messages shadow, derived from the simulation (dataset GT2
`f712b74643f5687f94d79356ca5cabec`, 62 vehicles: direct reconstruction, rear-to-front offset, and
geometric front) and not from the video. The video GTs (`ground_truth_rusia_v2_carril_processed.csv`,
`ground_truth_occupancy_rusia.csv`) remain the video's truth: they are distinct layers and do not
replace each other.

## Files

In `data/ground_truth/rusia/`:

- `gt_mensajes_sim_rusia_carril.csv` — route, lane, and crossing per vehicle (62 rows).
- `gt_mensajes_sim_rusia_occupancy.csv` — occupancy sibling (template from the video).
- `gt_mensajes_sim_rusia.README.md` — column-by-column contract.
- Reproducible GT generator (offline, not included): aborts if the dataset is not `f712b746…`.

## Shared measurement protocol

Every messages-shadow measurement compared against these GTs applies exactly these rules,
which are part of the experimental protocol and not GT details:

1. **Reference point = vehicle front**: the messages position (VEINS `getPosition`)
   is the vehicle front (the driver's rear-to-front offset). It matches the video GT's front rule:
   a uniform criterion throughout the project.
2. **Crossing detector = the net's stop line** (end of the approach edge, J5 side). The
   same geometry for GT and shadow.
3. **Committed crossing, with 3 m hysteresis and by transition**: a crossing counts when the
   transition from behind the line to in front is observed, and the front then advances ≥3 m without
   going back. Two mandatory consequences:
   - *Without hysteresis there is no valid count*: a queue head grazes the line (±0.5–1.3 m from the
     stop clamp) and would trigger the detector 30–100 s early (measured: v45 −100 s).
   - *By transition, not by state*: a vehicle whose first message is already in front of the line does
     not generate a crossing (event censored by the window). Counting by state would break the
     GT↔shadow symmetry (the GT censors v0/v1/v2; a by-state shadow would count them).
4. **Lane detector = appearance**: the SUMO lane of the front at insertion (straddling
   lanes, the one nearest the front wins, per SUMO's mapping rule). Verified: it matches the
   center lane in all 62. No bands, no fallback, no `sin_carril` (in the sim every vehicle
   is born in a lane).
5. **Time base = simulation seconds.** Crossing with the video GT: `t_sim = t_video + delay`
   (delay per vehicle in the dataset manifest).

## Special cases

The GT↔shadow symmetry is guaranteed:

- **Censored v0/v1/v2** (`cruce_censurado=si`, empty time): they are born with the front already past
  the line. v0 is born +5.07 m (out of band); v1 and v2 are born in band (+0.11/+0.01 m) and their
  commitment (≥3 m) is indeed observed (annotated in `notes` with its time, only for transparency).
  With the by-transition protocol, the shadow also generates no event for them, so the treatment is
  symmetric on both sides. In aggregate counts, both sides lose the same 3.
- **v9/v34/v76** (note in the CSV): they wait just past the net line because the painted stop line
  of the video is downstream in those approaches (line offset). They are not censored: their
  committed crossing is valid and the shadow, measuring over the same net line, sees the same
  event, so it is symmetric. The offset (−25 to −36 s) exists only against the video GT and
  is declared (it does not enter the shadow↔GT comparison).

## Cross-validation against the video GT

56 matchers, same clock (`cruce_sim − delay − entry_time_video`): median **−0.62 s** (p10 −1.73 /
p90 −0.12), consistent with a geometric front versus a human-annotated front. The exceptions
v9/v34/v76 are explained above.

## Provenance and stability

The generator runs the mobility driver's schedule. With the edge-removal driver (md5
`00778f15…`; the dataset `f712b746…` does not change), both GTs are **byte-identical** (md5 `be012d86…`
for lane and `3785aecc…` for occupancy): the driver's cutoff only affects samples after the end of the
exit edge, and all GT quantities live before that. Verified by regeneration and diff, not
assumed.

## Known limitations

Two thresholds of this protocol have a provisional value, without a sensitivity sweep:

- **Committed-crossing hysteresis: 3 m** (rule 3 of the protocol). Motivated by the measured case
  (queue heads grazing the line, ±0.5–1.3 m, would trigger the detector up to 100 s early), but the
  value has no count sensitivity sweep (FN/FP) against the threshold.
- **GT↔shadow matching tolerance window: 1.0 s** (`matching_tolerance_s`,
  `config/sites/rusia/site.yaml`). It is a provisional initial value: missing the sweep of artificial
  FN and FP against the tolerance.

Other parameters with a provisional value (`--lost-after-s` of 3 s and n=100 of the sweep) are
documented in `reconstruccion_causal/docs/reconstructor_causal.md`.

## References

- `data/ground_truth/rusia/gt_mensajes_sim_rusia.README.md`: column-by-column contract of the GTs.
- `docs/modules/matching_sombras.md`: shadow↔GT matching that consumes these GTs.
- `reconstruccion_causal/docs/reconstructor_causal.md`: causal reconstructor and sweep parameters.

# Module — Vehicle class stabilization (majority vote)

Assigns **one class per vehicle** (`vehicle_class` ∈ {car, bus, truck}) by **simple majority vote**
(the most frequent class, by number of frames) over the detections of the **reconnected** track. It
corrects **class flicker** (the same vehicle labeled car/truck/bus in different frames): at the
tracking stage it is measured that **~55 % of tracks change class** over their lifetime. It is
integrated in `scripts/derive_track_events.py` (the event layer gains the `vehicle_class` column); the
validation lives in `scripts/measure_class_accuracy.py`.

The vote is **simple, not weighted**, because it is the **standard MOT technique** (the tracker emits
the most frequent class over the track), defensible by citation. The weighted variants (by box area, by
confidence) were explored but showed no conclusive improvement with this sample (differences of 1–2
tracks over N=11 bus / N=4 truck, within the noise), so no custom variant is adopted without evidence.

The tracker is **class-agnostic**: it associates by IoU and `tracking/types.py` passes the `class_id`
of the per-frame detection without modifying it, and `class_id` is **not used** in the event or
matching logic. Stabilizing the class affects **only** the vehicle classification metric.

## Voting

Per track, `vehicle_class` = **mode** of `class_id` mapped to a name (the most frequent per frame). The
COCO id → name mapping is verified against `model.names` (yolov8x): **2=car, 5=bus, 7=truck**, and
matches `site.yaml class_set`. The detector already filters inference to those ids, so the vote
operates on the detector's raw classes. Ties (rare) are broken by the order of `value_counts`
(descending count). It returns `''` if the track has no class detections (does not happen in practice).

## Input and output

- **Input:** reconnected tracks (`class_id` per frame), `derive_track_events.CLASS_BY_COCO`.
- **Output:** the `vehicle_class` column (`"car"/"bus"/"truck"`) in `track_events.csv`, one per track.

## Measured validation (matched pairs, vs GT v2 `vehicle_class`)

GT v2: **58 car / 11 bus / 4 truck** (73, all with class; taxonomy 1:1 with the detector, 0
motorcycles). Matched: 69 (car 54, bus 11, truck 4).

- **Without vote:** per-frame accuracy **91 %**; stability: **55 % of tracks flicker**.
- **With vote:** accuracy **94 %**; stability **100 %** by construction (1 class per track).
- **Reference `always car`: 78 %** (the aggregate is inflated by cars).
- **Recall per class (deployed vote, Wilson 95% CI):**

  | class | recall | 95% CI | note |
  |---|---|---|---|
  | car | 53/54 = 98 % | [90, 100] | — |
  | bus | 9/11 = 82 % | [52, 95] | small N |
  | truck | 3/4 = 75 % | [30, 95] | **anecdotal (N=4)** |

  Confusion (rows GT, cols vote): car→{car 53, bus 1}; bus→{car 1, bus 9, truck 1}; truck→{car 1, truck 3}.
- **Explored variants (per-class):** simple 94 % (bus 9/11, truck 3/4); confidence 94 % (bus 10/11,
  truck 2/4); area 96 % (bus 10/11, truck 3/4). Differences within the noise, hence simple is the
  primary one.

The main benefit of stabilizing is **not accuracy** (per-frame is already 91 %, because cars dominate
and come out ~98 %) but **stability** (55 %→0 % flicker): a coherent class per vehicle. The accuracy
gain (~3 pts) is concentrated in bus/truck, where N is small and the CIs are wide.

## Sample caveat

The classes are heavily imbalanced (car 58 / bus 11 / truck 4). The aggregate is inflated by cars
(78 % would come from always saying "car"), so it is reported **per-class**. **truck N=4 is anecdotal**
(CI ~[30, 95 %]); bus N=11 also has a wide CI. The bus/truck numbers are **directional**.

## Residual flicker in the south

In visual verification, residual class flickers are observed in the **south**. **They are not
intra-track class instability** (the vote gives a constant class per track, confirmed on tid0): they
are the class-layer manifestation of the already-declared south limit —**unreconnected fragments** (a
vehicle split into several tracks, each with its own vote; jumping from one fragment to another changes
the `#id` and with it the displayed class)— plus **noisy raw class in the far field** (small boxes on
the horizon, so each fragment's vote is less reliable). It is the same south limit (far field plus
occlusion and fragmentation) documented in `track_events.md` and `lane_classifier.md`, not a new front.
Class stabilization does its job (one class per track); the residual would be addressed by south
fragment reconnection and, where applicable, duplicate suppression, not this layer.

## Limits

- It does not re-derive the approach-only baseline (preserved as a reference, without a class column);
  the class lives in the reconnected state.
- It does not adopt a weighted vote, for lack of evidence with this N.

## References

- `docs/modules/track_events.md`: the event layer that gains the `vehicle_class` column.
- `docs/modules/lane_classifier.md`: lane classifier of the same event layer.
- `docs/modules/reconnect_fragments.md`: the fragment reconnection that the vote operates on.
- Site config: `config/sites/rusia/site.yaml` (`class_set`).

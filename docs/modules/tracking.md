# Module — Tracking (`digital_shadow.tracking`)

Maintains the **stable identity** of each vehicle over time, so its route can be reconstructed.
It is a consumer of the detector: it receives `list[Detection]` per frame and emits
`list[TrackedDetection]` with `track_id`. It knows nothing about source, pacing, dropping or zones (the
same clean boundary as the rest of the pipeline). The timestamp is matched by the driver from the
`Frame` of the source. The algorithm is **ByteTrack**, with the implementation
**`trackers.ByteTrackTracker`** (Roboflow).

## Files and loop

```
digital_shadow/tracking/
  types.py    TrackedDetection + conversions with sv.Detections
  tracker.py  Tracker (ABC) + ByteTracker (wraps trackers.ByteTrackTracker)
config/tracker.yaml          tracker thresholds (site-independent)
config/sites/<id>/site.yaml  lost_track_buffer_frames (site-dependent)
```

Operating loop: `source.read() → detector.detect(frame.image) → tracker.update(dets)`.

## The three architecture decisions

Each decision is documented with its rationale so a reviewer can trace it.

### Tracker = ByteTrack (algorithm)

ByteTrack associates **all** the boxes, including the low-confidence ones, in two stages: first the
high-confidence ones with the tracks (Kalman + IoU), then the tracks still unmatched with the
**low**-confidence boxes by IoU
([Zhang et al., 2022, arXiv 2110.06864](https://arxiv.org/abs/2110.06864)). This directly attacks
the site's dominant failure: the visual validation of the detection showed that the
escapes occur by occlusion behind buses with reappearance (confidence drops during the occlusion).
The distinguishing mechanisms of BoT-SORT do not apply here:

- **Camera motion compensation (CMC):** it presupposes a moving camera; ours is fixed, so
  it is superfluous (cost without benefit, possible noise).
- **Appearance re-identification:** overhead vehicles of the same class are almost identical,
  a weak and expensive signal ([Aharon et al., 2022, arXiv 2206.14651](https://arxiv.org/abs/2206.14651)).

BoT-SORT without those two is equivalent to ByteTrack, so it is cleaner to choose ByteTrack and not drag
dead machinery. Besides, the metric is the **route**, not the fine position, so the more precise
Kalman of BoT-SORT is not needed either.

An ablation confirms it: **OC-SORT** (non-linear motion axis, also in
`trackers`) was tested against ByteTrack on the bus turn events. OC-SORT did not recover them (the
turn fragmentations stayed broken) and it regressed on occlusions that ByteTrack held (10/23
vs 15/23 of control), with more crossings (6 vs 2). The motion axis is exhausted for this failure;
the path is appearance re-identification (optional layer described below). The tracker
is selectable by config (`tracker_type`) in case it is re-evaluated, but ByteTrack is kept
fixed.

### Integration = decoupled tracker `trackers.ByteTrackTracker`

- **Not Ultralytics `model.track()`:** it re-runs the detection internally
  ([Ultralytics docs](https://docs.ultralytics.com/modes/track)), duplicating the single source of
  detections and breaking the source's pacing/dropping. The tracker is one more stage that
  consumes the `(N,6)` detections.
- **`trackers.ByteTrackTracker`, not `sv.ByteTrack`:** the initial decision was
  `supervision.ByteTrack`, but `sv.ByteTrack` was deprecated in supervision 0.28.0
  (`FutureWarning: deprecated since v0.28.0, will be removed in v0.30.0`). Roboflow moved the
  tracking to its dedicated library **`trackers`**
  ([roboflow/trackers](https://github.com/roboflow/trackers);
  [trackers.roboflow.com](https://trackers.roboflow.com/latest/trackers/bytetrack/)),
  actively maintained (2.4.0), Apache-2.0, with clean dependencies and using `sv.Detections`
  as I/O format. That is why `trackers==2.4.0` is pinned for the tracking and
  `supervision==0.28.0` is kept only for the annotators of the visual validation. Building on a
  deprecated API would be fragile for something a reviewer must reproduce.

### Operational detector confidence ~0.10 (single regime)

ByteTrack needs the low-confidence boxes: its second association stage (the one that recovers
occlusions) works between ~0.1 and the high threshold. If the detector filters at 0.25, it removes exactly
the detections it needs to solve the problem it was chosen for. That is why the detector emits at
**conf 0.10** as its single operational regime (`config/detector.yaml`). The `0.25` of the detection
was a visual presentation threshold, not operational: if a visualization tool wants only
strong boxes, it filters in the visualization, not in the detector (which stays "dumb"). The ~0.1
is ByteTrack's default `track_low_thresh`
([Ultralytics bytetrack.yaml](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/trackers/bytetrack.yaml)):
a reasoned starting point, subject to sensitivity analysis against the GT.

## Output contract (raw trajectories)

`TrackedDetection` (frozen): `track_id: int`, `x1,y1,x2,y2: float` (**raw** box in px, same
as the detector — the derivation to the plane is done by the homography in the zones layer),
`class_id: int`, `confidence: float`, and properties `cx,cy` (box center, only a convenience
for visualization; it is not the homography reference point).

**Only confirmed tracks are emitted (`track_id >= 0`).** The backend marks with `-1` the
tentative unconfirmed tracks; `sv_to_tracked` discards them. Reason: a tentative track that never
confirms is noise that would contaminate the route inference; real vehicle trajectories are wanted,
not candidates. Consequence: the first frame of each track (tentative) is not emitted; the
trajectory starts one frame after the first detection (negligible for the route).

**Persistent artifact** (`scripts/run_tracker.py` → CSV, what the zones layer consumes), one
row per track observation per frame: `timestamp_s, frame_index, track_id, x1, y1, x2, y2,
cx, cy, class_id, confidence`. Without zones, approaches or routes; that is the zones layer.

## Parameters (site/pipeline separation)

**`config/tracker.yaml` (site-independent):**

| Parameter | Value | Mapping / note |
|---|---|---|
| `track_activation_threshold` | 0.25 | threshold to start a new track (≈ new_track_thresh) |
| `high_conf_det_threshold` | 0.25 | high/low boundary of the two stages (≈ track_high_thresh); reconciles with the detector's conf 0.10 |
| `minimum_iou_threshold` | 0.2 | minimum IoU to associate |
| `minimum_consecutive_frames` | 1 | frames to confirm (anti-flicker); raised to 2–3 only if 1-frame track inflation is observed |

**`config/sites/<id>/site.yaml` (site-dependent):**

| Parameter | Value (Rusia) | Note |
|---|---|---|
| `lost_track_buffer_frames` | 140 | frames a track is retained without matching; tuned by the site's occlusion (see "Tuning of `lost_track_buffer`") |
| `frame_rate` | — | derived from `fps_ref` (29.583), not duplicated |

`lost_track_buffer_frames` lives in `site.yaml` (not in `tracker.yaml`) because by definition
it depends on the site: it is fixed by this site's occlusion duration. The architecture standard is
that whatever is site-dependent goes in the site config, so that another site with another occlusion does not
break replicability.

## Tuning of `lost_track_buffer`

Method (`scripts/measure_occlusion.py` + `scripts/inspect_occlusion_tail.py`):

1. **Measurement:** distribution of bridged gaps per track (frames that a same `track_id`
   was occluded and reappeared with the same id), with a generous buffer.
2. **Re-examination of the tail:** the long gaps are separated by the centroid displacement
   between before/after the gap:
   - **displ ≈ 0 px → static objects** (parking lot/queue) flickering; their gaps reach
     205 frames (6.9 s) but they are not traffic occlusion, so they do not size the buffer (and the
     zones layer excludes the parking lot).
   - **real displ → moving vehicles** occluded by buses; confirmed maximum 131 frames
     (4.43 s) (a truck; another event confirmed visually with the bus over the corridor).
     It matches the ~4 s of occlusion observed in the video.
3. **Fixing:** 140 frames (4.73 s) = real moving maximum (131) + ~7 %; below the
   static gaps (150–205) to limit crossed identities.
4. **Validation:** in the regenerated video, (a) ID survival in occlusions and
   (b) absence of crossings are inspected; objectively with `scripts/analyze_track_continuity.py` (extrapolation
   error of the reappearance).

**Interpretation caveat:** treating the maximum (131 frames) as an "erroneous association" leads to an
undersized buffer (≈p99, 75 frames). The visual evidence (occlusions of ~4 s) and the
re-examination by displacement show that this maximum is a real occlusion of a moving vehicle:
the high tail of the distribution is not noise, and separating moving/static by
displacement is what interprets it correctly.

## Assumption to watch: uniform cadence

ByteTrack's Kalman assumes uniform time steps, and `lost_track_buffer` counts **frames**.
With 0 % dropping (the canonical run re-verifies it with detector+tracker) the cadence is
uniform and the assumption holds. If the dropping rose, non-uniform time jumps would
distort the tracking and the meaning in seconds of the buffer. It is an assumption to watch, not
a current problem.

## Tracking / zones-layer boundary

The tracking delivers raw trajectories (this module plus the CSV). The conversion
trajectory → event (entry_arm/exit_arm/turn) and the official route metric against the GT belong to
the zones layer (the same one that excludes the parking lot). No zones are anticipated here.

## Validation (tracking quality, not route fidelity)

- **Visual** (`scripts/annotate_tracks.py`): video with boxes, `#id class` and trajectory, stable color
  per `track_id`. The reviewer judges whether the id holds through the crossing and whether it survives the
  occlusion behind a bus.
- **Sanity check** (`scripts/run_tracker.py`, explicitly labeled as such, not as a fidelity
  metric): number of unique tracks compared in order of magnitude with the **72** of the GT. The
  count is inflated by the parking lot (not yet excluded, a task of the zones layer) and by
  ID fragmentation; the fine number is not interpretable until the zones layer.

## Usage

```python
from digital_shadow.config import load_site_config, load_detector_config, load_tracker_config
from digital_shadow.perception import YoloDetector
from digital_shadow.sources import file_source_from_site
from digital_shadow.tracking import byte_tracker_from_config

site = load_site_config("rusia")
detector = YoloDetector(load_detector_config(), list(site.class_set)).load()
detector.warmup(site.frame_height, site.frame_width)
tracker = byte_tracker_from_config(load_tracker_config(), site)   # one instance/session

with file_source_from_site(site) as source:
    while True:
        frame = source.read()
        if frame.end_of_source:
            break
        tracked = tracker.update(detector.detect(frame.image))   # list[TrackedDetection]
        # driver matches frame.timestamp_s with `tracked`
```

## Optional re-id layer — `digital_shadow/tracking/reid/`

Layer for reconnecting tracklets by appearance, after ByteTrack (ByteTrack intact), for
the bus turns that motion does not recover (occlusion ≥4 s + maneuver). Config switch:
with `reid.enabled:false` it is `NoOpReconnector`, a bit-identical passthrough to the base ByteTrack
(verified by CSV `diff`), which gives a clean ablation. With ON, `ReidReconnector` maintains
a gallery of lost tracks and, when a new id is born, compares it by embedding cosine
only against the lost ones gated in time and space; it reconnects if it exceeds `similarity_threshold`
and beats the 2nd candidate by `similarity_margin` (when in doubt, do not reconnect: a crossing is worse than a
fragmentation). Embeddings from clean frames (birth + refresh every
`embedding_refresh_frames`, not per-frame). Identical output contract (reassigned track_id), so
the zones layer does not change.

- **Seam `EmbeddingModel.extract(boxes, frame) -> (N,D)`:** an interchangeable piece. Today
  `OsnetEmbeddingModel` (generic OSNet via **boxmot**; `torchreid` from PyPI was discarded due to
  dead dependencies with torch 2.12). A `CustomEmbeddingModel` (fine-tuned) is activated by
  changing `reid.model`, without touching the reconnector.
- **Parameters:** model + thresholds in `tracker.yaml` (`reid:`); gates
  `reid_temporal_window_frames` / `reid_spatial_corridor_px` in `site.yaml` (they depend on the
  site's occlusion, same rule as `lost_track_buffer`).
- **Floor measurement with generic OSNet:** at the default (x1_0 @ 0.6) it reconnects 15/123 turns
  (vs 1 baseline, 3 OC-SORT) but with crossings 2→11 and ~9 % dropping. A full threshold sweep
  (0.3–0.7) × model (x1_0, x0_25) + margin: reconnections and crossings are coupled (both dials
  slide along the same boundary); there is no net-positive point. At most ~20–28 % of turns
  reconnected with crossings ≈ reconnections (worse than fragmenting for the shadow). x0_25 costs 4×
  less dropping (2.47 % vs 8.44 %) with comparable reconnection, but it is not profitable either.
  Verdict: the architecture works; the generic model is the bottleneck (overhead almost identical),
  so re-id stays OFF by default; the real lever is the custom fine-tuned model (the seam
  is ready), not more tuning of the generic one.

## References

- `docs/modules/detector.md`: the detector that the tracker consumes (conf 0.10).
- `docs/modules/video_source.md`: the source, below the detector.
- ByteTrack: [arXiv 2110.06864](https://arxiv.org/abs/2110.06864) · BoT-SORT:
  [arXiv 2206.14651](https://arxiv.org/abs/2206.14651) · Ultralytics tracking:
  [docs](https://docs.ultralytics.com/modes/track) · trackers:
  [trackers.roboflow.com](https://trackers.roboflow.com/).
- Environment: Python 3.10.12, `trackers==2.4.0`, `supervision==0.28.0`. Dependencies pinned in
  `requirements.txt`.

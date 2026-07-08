# Module — Detector (`digital_shadow.perception`)

Detects vehicles in each frame with YOLOv8x (pretrained COCO weights). It acts as a
consumer of the video source: it asks for a frame, processes it, and asks for the next. It
does not manage frame dropping; the source governs it: the longer `detect()` takes, the more
frames the source drops on the next read. The boundary is clean: the detector receives an
image and returns boxes; it does not know about pacing, the source, or the timestamp (the
driver pairs it up).

## Files

```
digital_shadow/perception/
  types.py      Detection (dataclass) + as_array() (ByteTrack format)
  coco.py       names_to_ids(): class names -> COCO ids (from the model)
  detector.py   Detector (ABC) + YoloDetector (Ultralytics)
config/detector.yaml         model and inference parameters (site-independent)
config/sites/<id>/site.yaml  class_set: active class filter (site-dependent)
```

## Input and output

`Detector.detect(image: np.ndarray) -> list[Detection]`. It receives a BGR image (the one
the source delivers) and returns the list of detections for the frame. The timestamp is added by
the driver from the source's `Frame`: the per-frame output —detections plus timestamp— is
materialized as `(timestamp_s, [Detection...])` at the driver level, not inside the
detector.

`Detection` (dataclass, frozen):
- `x1, y1, x2, y2` (float): corners in image pixels, raw; not a derived
  reference point (that is done by the homography later).
- `confidence` (float).
- `class_id` (int): canonical COCO id, the cross key with ground truth.
- `class_name` (str): only for visualization; the id is the key.

`as_array(detections) -> np.ndarray` of shape `(N, 6)`, with rows
`[x1, y1, x2, y2, conf, class_id]`, the format that ByteTrack/BoT-SORT expect in
tracking. With `N = 0` it returns `(0, 6)`.

## Classes: the site filter

The detector emits COCO ids; the site defines its filter by names (`class_set` in
`site.yaml`). `coco.names_to_ids` translates names to ids using the loaded model's own
`model.names` map, which is the authoritative source; no COCO id is hardcoded in the
code. At the Rusia crossing, `class_set = [car, bus, truck]` translates to ids
`[2, 5, 7]`, which the detector passes as `classes=[...]` to YOLO so as not to spend compute on
the other classes.

`motorcycle` (id 3) remains in the COCO vocabulary; it is only out of Rusia's active
filter, whose ground truth has no motorcycles. Another site with motorcycles reactivates them via its
`class_set`, without touching code.

## Parameters (`config/detector.yaml`, site-independent)

| Parameter | Value | Note |
|---|---|---|
| `model.weights` | `yolov8x.pt` | changing model amounts to changing this line |
| `model.weights_dir` | `models` | where the weights live or are downloaded |
| `inference.confidence` | `0.10` | low operating threshold: feeds ByteTrack's second association stage (a presentation threshold would be ~0.25) |
| `inference.iou` | `0.7` | IoU threshold of the intra-class NMS (Ultralytics default) |
| `inference.agnostic_nms` | `true` | cross-class NMS (see the next section) |
| `inference.imgsz` | `640` | inference side |
| `inference.device` | `cuda:0` | inference device |
| `inference.half` | `true` | FP16, for real time; introduces non-determinism: two runs are not byte-exact |

## Cross-class NMS (`agnostic_nms`)

The Ultralytics default NMS is intra-class: it suppresses overlapping boxes only within
the same class. A large vehicle, such as a bus, is often detected both as bus and as
truck in nearly identical boxes; the intra-class NMS lets both through, ByteTrack creates two
concurrent ids over the same vehicle, and a duplicate is drawn in the shadow. With
`agnostic_nms=True` the NMS runs over all classes together and suppresses the lower-confidence
box.

On the Rusia video, activating it suppresses 4138 boxes crossed between classes (confidence
0.10–0.34, IoU 0.70–0.85 with the box that suppresses them) without losing any box of a
real vehicle: the NMS never removes a box without another of higher confidence overlapping it, and
two distinct nearby vehicles have a box IoU near zero. The effect in the
shadow is to reduce the drawn duplicates from 11 to 1.

Scope: it resolves the cross-class duplicates (bus and truck). It does not resolve the
same-class duplicates —two boxes of the same bus with high IoU, or partial boxes—, which
correspond to the tracker and the detector respectively.

## Model weights

`yolov8x.pt` (~131 MB) is downloaded the first time as an Ultralytics asset and normalized
to `models/` so that the download is idempotent. It is not versioned, being a large,
downloadable binary; its traceability is by SHA256 and size. Pinning the `ultralytics`
version determines which `yolov8x.pt` is downloaded.

## Usage

```python
from digital_shadow.config import load_site_config, load_detector_config
from digital_shadow.perception import YoloDetector, as_array
from digital_shadow.sources import file_source_from_site

site = load_site_config("rusia")
detector = YoloDetector(load_detector_config(), list(site.class_set)).load()
detector.warmup(site.frame_height, site.frame_width)   # pays the CUDA initialization

with file_source_from_site(site) as source:
    while True:
        frame = source.read()
        if frame.end_of_source:
            break
        detections = detector.detect(frame.image)   # if it is slow, the source drops
        boxes = as_array(detections)                 # (N, 6) for the tracker
```

## Real-time performance

Real-time viability is summarized in the drop rate: the detector consumes the
source in real time up to the end of the video and how many frames it drops out of the
total is reported. The detector is warmed up before opening the source so that the CUDA
initialization does not count as dropping. The reference figures were measured on an RTX 4080 Laptop
with FP16 and `imgsz = 640`.

## Known limitations

- **COCO class noise**: confusion between bus, truck, and van, and to a lesser degree between
  car and truck. It is irrelevant for this work's metric, which is the route (origin,
  destination, and turn), not the fine class: a misclassified vehicle enters and exits through the
  same approaches. The classes are not merged.
- **Occlusion**: a vehicle blocked by a bus is not detected in those frames; tracking recovers it.
- **Box flicker** in some vehicles: inherent to per-frame detection; tracking
  stabilizes it.
- **FP16**: a deliberate decision for real-time deployment; introduces non-determinism
  between runs.
- **Parked vehicles**: the video includes vehicles in a parking lot that the detector
  sees correctly but that are not intersection traffic. The detector stays without
  zone logic —it detects everything it sees— to remain clean and reusable across sites;
  the exclusion is applied in the zones layer, after the detector and before tracking.

## References

- `digital_shadow.sources` (`docs/modules/video_source.md`): the source the detector consumes.
- Dependencies and interpreter: `requirements.txt`.

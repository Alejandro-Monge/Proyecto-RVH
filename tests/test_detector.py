"""Deterministic tests of the detector's pure logic (no model, no GPU).

The real YOLO inference is exercised by the scripts over the video; here we
verify in milliseconds the logic that does not depend on the model: the class
name→id COCO mapping (including the exclusion of motorcycle in Rusia), parsing an
Ultralytics result into a `Detection`, the `(N,6)` format for ByteTrack, and
loading `detector.yaml`. Runnable with pytest or as a script.
"""

from __future__ import annotations

import numpy as np

from digital_shadow.config import load_detector_config
from digital_shadow.perception.coco import names_to_ids
from digital_shadow.perception.detector import YoloDetector
from digital_shadow.perception.types import Detection, as_array

# Subset of the real COCO vocabulary (id→name) that the model ships with.
COCO_NAMES = {2: "car", 3: "motorcycle", 5: "bus", 7: "truck", 0: "person"}


# --------------------------------------------------------------------------- #
# Test doubles
# --------------------------------------------------------------------------- #
class FakeBoxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float32)
        self.conf = np.asarray(conf, dtype=np.float32)
        self.cls = np.asarray(cls, dtype=np.float32)

    def __len__(self):
        return len(self.cls)


class FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


# --------------------------------------------------------------------------- #
# Class mapping
# --------------------------------------------------------------------------- #
def test_class_set_to_coco_ids_excludes_motorcycle() -> None:
    ids = names_to_ids(["car", "bus", "truck"], COCO_NAMES)
    assert ids == [2, 5, 7]
    assert 3 not in ids  # motorcycle outside Rusia's active filter


def test_unknown_class_name_raises() -> None:
    try:
        names_to_ids(["car", "bicicleta"], COCO_NAMES)
    except ValueError:
        pass
    else:
        raise AssertionError("una clase desconocida debió lanzar ValueError")


def test_class_ids_preserve_order_and_dedup() -> None:
    assert names_to_ids(["truck", "car", "car"], COCO_NAMES) == [7, 2]


# --------------------------------------------------------------------------- #
# Result parsing
# --------------------------------------------------------------------------- #
def _detector_with_names() -> YoloDetector:
    cfg = load_detector_config()  # reads the real config/detector.yaml
    det = YoloDetector(cfg, ["car", "bus", "truck"])
    det._model_names = COCO_NAMES  # inject what the loaded model would give
    return det


def test_parse_result_to_detections() -> None:
    det = _detector_with_names()
    boxes = FakeBoxes(
        xyxy=[[10, 20, 110, 220], [5, 5, 15, 35]],
        conf=[0.91, 0.42],
        cls=[2, 5],  # car, bus
    )
    detections = det._parse(FakeResult(boxes))

    assert len(detections) == 2
    d0 = detections[0]
    assert (d0.x1, d0.y1, d0.x2, d0.y2) == (10.0, 20.0, 110.0, 220.0)
    assert abs(d0.confidence - 0.91) < 1e-5  # FakeBoxes uses float32
    assert d0.class_id == 2 and d0.class_name == "car"
    assert detections[1].class_id == 5 and detections[1].class_name == "bus"


def test_parse_empty_result() -> None:
    det = _detector_with_names()
    boxes = FakeBoxes(xyxy=np.empty((0, 4)), conf=[], cls=[])
    assert det._parse(FakeResult(boxes)) == []


# --------------------------------------------------------------------------- #
# ByteTrack format
# --------------------------------------------------------------------------- #
def test_as_array_format() -> None:
    dets = [
        Detection(10, 20, 110, 220, 0.91, 2, "car"),
        Detection(5, 5, 15, 35, 0.42, 5, "bus"),
    ]
    arr = as_array(dets)
    assert arr.shape == (2, 6)
    np.testing.assert_allclose(arr[0], [10, 20, 110, 220, 0.91, 2], rtol=1e-6)
    np.testing.assert_allclose(arr[1], [5, 5, 15, 35, 0.42, 5], rtol=1e-6)


def test_as_array_empty_is_2d() -> None:
    arr = as_array([])
    assert arr.shape == (0, 6)


# --------------------------------------------------------------------------- #
# Detector config
# --------------------------------------------------------------------------- #
def test_detector_config_loads() -> None:
    cfg = load_detector_config()
    assert cfg.weights == "yolov8x.pt"
    assert cfg.half is True  # FP16 from the start
    assert cfg.confidence == 0.10  # OPERATIONAL conf (candidates for ByteTrack)
    assert cfg.weights_path.name == "yolov8x.pt"
    assert cfg.weights_path.parent.name == "models"


def test_detect_before_load_raises() -> None:
    cfg = load_detector_config()
    det = YoloDetector(cfg, ["car"])
    try:
        det.detect(np.zeros((10, 10, 3), dtype=np.uint8))
    except RuntimeError:
        pass
    else:
        raise AssertionError("detect() antes de load() debió lanzar RuntimeError")


def _run_all() -> None:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"OK  {fn.__name__}")
    print(f"\n{len(tests)} tests deterministas del detector pasaron.")


if __name__ == "__main__":
    _run_all()

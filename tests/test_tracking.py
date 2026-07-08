"""Deterministic tests of the tracking module.

They verify the logic of our wrapper and the property we care about —occlusion
recovery via `lost_track_buffer`— deterministically (the ByteTrack backend is
deterministic given the input). The real YOLO inference is not involved: the
tracker is fed synthetic detections.

Note: the backend's track_id counter is per-process (it does not reset between
instances), so the assertions are RELATIONAL (stable id / reappearing id == prior
id), never absolute values. Runnable with pytest or as a
script.
"""

from __future__ import annotations

import numpy as np
import supervision as sv

from digital_shadow.config import ReidSettings, TrackerConfig
from digital_shadow.perception.types import Detection
from digital_shadow.tracking import (
    ByteTracker,
    TrackedDetection,
    detections_to_sv,
    sv_to_tracked,
)

# Test config: thresholds that confirm tracks from conf-0.9 boxes; small buffer
# (5 frames) and frame_rate=30 so the buffer is ~5 frames exactly.
_REID_OFF = ReidSettings(
    enabled=False, model="osnet_x1_0_msmt17", device="cpu", half=False,
    similarity_threshold=0.6, similarity_margin=0.05, embedding_refresh_frames=15,
)
TEST_CFG = TrackerConfig(
    tracker_type="bytetrack",
    high_conf_det_threshold=0.25,
    minimum_iou_threshold=0.2,
    minimum_consecutive_frames=1,
    track_activation_threshold=0.25,
    delta_t=3,
    direction_consistency_weight=0.2,
    reid=_REID_OFF,
)
BOX = (100.0, 200.0, 160.0, 260.0)


def _det(box=BOX, conf=0.9, cid=2) -> Detection:
    return Detection(box[0], box[1], box[2], box[3], conf, cid, "car")


def _new_tracker(lost_buffer: int = 5) -> ByteTracker:
    return ByteTracker(TEST_CFG, frame_rate=30.0, lost_track_buffer=lost_buffer)


# --------------------------------------------------------------------------- #
# detector <-> tracker conversion
# --------------------------------------------------------------------------- #
def test_detections_to_sv_roundtrip_fields() -> None:
    sv_dets = detections_to_sv([_det(box=(10, 20, 110, 220), conf=0.8, cid=5)])
    assert sv_dets.xyxy.shape == (1, 4)
    np.testing.assert_allclose(sv_dets.xyxy[0], [10, 20, 110, 220])
    assert abs(float(sv_dets.confidence[0]) - 0.8) < 1e-6
    assert int(sv_dets.class_id[0]) == 5


def test_detections_to_sv_empty() -> None:
    sv_dets = detections_to_sv([])
    assert len(sv_dets) == 0


def test_sv_to_tracked_drops_tentative_ids() -> None:
    # Two boxes: one tentative (-1) and one confirmed (id 7). Only the confirmed one comes out.
    dets = sv.Detections(
        xyxy=np.array([[0, 0, 10, 10], [20, 20, 40, 40]], dtype=float),
        confidence=np.array([0.9, 0.9]),
        class_id=np.array([2, 5]),
    )
    dets.tracker_id = np.array([-1, 7])
    tracked = sv_to_tracked(dets)
    assert len(tracked) == 1
    assert tracked[0].track_id == 7
    assert tracked[0].class_id == 5


# --------------------------------------------------------------------------- #
# Tracker behavior
# --------------------------------------------------------------------------- #
def test_stable_id_while_visible() -> None:
    tracker = _new_tracker()
    confirmed_ids = []
    for _ in range(6):
        tracked = tracker.update([_det()])
        if tracked:
            confirmed_ids.append(tracked[0].track_id)
    # After confirmation, a single stable id (>=0) while the vehicle is visible.
    assert len(confirmed_ids) >= 4
    assert len(set(confirmed_ids)) == 1
    assert confirmed_ids[0] >= 0


def test_short_occlusion_keeps_id() -> None:
    # Occlusion SHORTER than the buffer (3 < 5): the id is kept on reappearance.
    tracker = _new_tracker(lost_buffer=5)
    for _ in range(6):
        tracked = tracker.update([_det()])
    pre_gap_id = tracked[0].track_id
    for _ in range(3):  # gap < buffer
        tracker.update([])  # frame with no detections (update called anyway)
    after = tracker.update([_det()])
    assert len(after) == 1
    assert after[0].track_id == pre_gap_id  # bridged: same id


def test_long_occlusion_switches_id() -> None:
    # Occlusion LONGER than the buffer (12 > 5): NOT bridged -> the prior id does
    # not reappear (new id, which for the route metric fragments the vehicle).
    tracker = _new_tracker(lost_buffer=5)
    for _ in range(6):
        tracked = tracker.update([_det()])
    pre_gap_id = tracked[0].track_id
    for _ in range(12):  # gap > buffer
        tracker.update([])
    after = tracker.update([_det()])
    after_id = after[0].track_id if after else None
    assert after_id != pre_gap_id  # not bridged


def test_empty_frame_returns_empty_and_is_callable() -> None:
    tracker = _new_tracker()
    assert tracker.update([]) == []  # frame with no detections breaks nothing


def test_reset_returns_self() -> None:
    tracker = _new_tracker()
    tracker.update([_det()])
    assert tracker.reset() is tracker


def test_update_returns_tracked_detection_type() -> None:
    tracker = _new_tracker()
    tracker.update([_det()])
    tracked = tracker.update([_det()])
    assert tracked and isinstance(tracked[0], TrackedDetection)
    # raw box preserved; derived center correct
    td = tracked[0]
    assert (td.x1, td.y1, td.x2, td.y2) == BOX
    assert td.cx == 0.5 * (BOX[0] + BOX[2])
    assert td.cy == 0.5 * (BOX[1] + BOX[3])


# --------------------------------------------------------------------------- #
# OC-SORT (same contract) and selection factory
# --------------------------------------------------------------------------- #
def test_ocsort_stable_id_and_bridges_short_occlusion() -> None:
    from digital_shadow.tracking import OcSortTracker

    tracker = OcSortTracker(TEST_CFG, frame_rate=30.0, lost_track_buffer=10)
    last = []
    for _ in range(8):  # OC-SORT confirms in >=3 frames by its mechanism
        last = tracker.update([_det()])
    assert last and last[0].track_id >= 0
    pre = last[0].track_id
    for _ in range(3):  # occlusion < buffer
        tracker.update([])
    after = tracker.update([_det()])
    assert after and after[0].track_id == pre  # same contract, bridges


def test_tracker_from_config_dispatch() -> None:
    from types import SimpleNamespace

    from digital_shadow.tracking import ByteTracker, OcSortTracker, tracker_from_config

    site = SimpleNamespace(fps_ref=30.0, lost_track_buffer_frames=10)
    assert isinstance(tracker_from_config(TEST_CFG, site), ByteTracker)
    assert isinstance(
        tracker_from_config(TEST_CFG, site, override_type="ocsort"), OcSortTracker
    )
    try:
        tracker_from_config(TEST_CFG, site, override_type="nope")
    except ValueError:
        pass
    else:
        raise AssertionError("tracker_type desconocido debió lanzar ValueError")


def _run_all() -> None:
    import warnings

    warnings.simplefilter("ignore")
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"OK  {fn.__name__}")
    print(f"\n{len(tests)} tests deterministas del tracking pasaron.")


if __name__ == "__main__":
    _run_all()

"""Tracking output structures and conversions with supervision.

`TrackedDetection` is a detection with stable identity (`track_id`). It carries
the RAW box (same as the detector): the derivation to the road plane is done by
the homography downstream. The emitted trajectories contain ONLY CONFIRMED tracks
(`track_id >= 0`): a tentative track (id -1) that never confirms is noise that
would contaminate the route inference; we want real vehicles, not candidates.

The exchange format with the tracking backend is supervision's `sv.Detections`
(what `trackers.ByteTrackTracker` consumes and returns).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import supervision as sv

from ..perception.types import Detection


@dataclass(frozen=True)
class TrackedDetection:
    """A detection with stable track identity."""

    track_id: int  # stable vehicle id (>= 0; the tentative -1 ones are not emitted)
    x1: float
    y1: float
    x2: float
    y2: float
    class_id: int  # COCO id
    confidence: float

    @property
    def cx(self) -> float:
        """Box center X (visualization convenience; NOT the homography reference
        point, which is derived downstream)."""
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        """Box center Y (visualization convenience; see `cx`)."""
        return 0.5 * (self.y1 + self.y2)


def detections_to_sv(detections: list[Detection]) -> sv.Detections:
    """Convert the detector output into `sv.Detections` for the tracker."""
    if not detections:
        return sv.Detections.empty()
    xyxy = np.array([[d.x1, d.y1, d.x2, d.y2] for d in detections], dtype=float)
    confidence = np.array([d.confidence for d in detections], dtype=float)
    class_id = np.array([d.class_id for d in detections], dtype=int)
    return sv.Detections(xyxy=xyxy, confidence=confidence, class_id=class_id)


def tracked_to_sv(tracked: list[TrackedDetection]) -> sv.Detections:
    """Convert trajectories into `sv.Detections` (with tracker_id) for annotation.

    Symmetry with `detections_to_sv`: the visualization consumes the tracking
    output contract, not internal backend state.
    """
    if not tracked:
        return sv.Detections.empty()
    dets = sv.Detections(
        xyxy=np.array([[t.x1, t.y1, t.x2, t.y2] for t in tracked], dtype=float),
        confidence=np.array([t.confidence for t in tracked], dtype=float),
        class_id=np.array([t.class_id for t in tracked], dtype=int),
    )
    dets.tracker_id = np.array([t.track_id for t in tracked], dtype=int)
    return dets


def sv_to_tracked(tracked: sv.Detections) -> list[TrackedDetection]:
    """Convert the tracker output into `TrackedDetection`, ONLY confirmed ones.

    Filters the tentative track_ids (-1): only vehicles with confirmed identity
    are emitted. If the tracker assigned no ids (frame with no tracks), returns [].
    """
    if tracked.tracker_id is None or len(tracked) == 0:
        return []
    result: list[TrackedDetection] = []
    confidences = (
        tracked.confidence
        if tracked.confidence is not None
        else np.full(len(tracked), np.nan)
    )
    class_ids = (
        tracked.class_id
        if tracked.class_id is not None
        else np.full(len(tracked), -1, dtype=int)
    )
    for (x1, y1, x2, y2), tid, conf, cid in zip(
        tracked.xyxy, tracked.tracker_id, confidences, class_ids
    ):
        if tid < 0:  # unconfirmed tentative track -> not emitted
            continue
        result.append(
            TrackedDetection(
                track_id=int(tid),
                x1=float(x1),
                y1=float(y1),
                x2=float(x2),
                y2=float(y2),
                class_id=int(cid),
                confidence=float(conf),
            )
        )
    return result

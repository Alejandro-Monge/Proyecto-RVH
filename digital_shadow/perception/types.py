"""Detector output structures.

`Detection` carries RAW corners (x1,y1,x2,y2) in image pixels, without a derived
reference point: the derivation to the road plane is done by the homography later. The
`(N,6)` format `[x1,y1,x2,y2,conf,class_id]` is the one ByteTrack/BoT-SORT expect.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Detection:
    """A box detected in a frame."""

    # Corners in image pixels (top-left and bottom-right corner).
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int  # canonical COCO id (join key with the ground truth)
    class_name: str  # only for visualization/legibility; the id is the key

    def as_row(self) -> list[float]:
        """Row `[x1,y1,x2,y2,conf,class_id]` (ByteTrack/BoT-SORT format)."""
        return [self.x1, self.y1, self.x2, self.y2, self.confidence, float(self.class_id)]


def as_array(detections: list[Detection]) -> np.ndarray:
    """Pack detections into an `ndarray` `(N,6)` `[x1,y1,x2,y2,conf,class_id]`.

    With N=0 it returns an array of shape `(0,6)` (the trackers expect 2D even when there
    are no detections).
    """
    if not detections:
        return np.empty((0, 6), dtype=np.float32)
    return np.asarray([d.as_row() for d in detections], dtype=np.float32)

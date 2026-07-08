"""Tracking: contract and implementations swappable by config.

The tracker is a CONSUMER of the detector: it receives `list[Detection]` per frame
and emits `list[TrackedDetection]` with stable identity. It knows NOTHING about
source, pacing, discard, or zones (same clean boundary as the rest of the
pipeline). The timestamp is matched by the driver (it comes in the source's
`Frame`).

The concrete tracker is selected by config (`tracker_type`): **ByteTrack** (the
fixed one) and **OC-SORT** (experimental, non-linear motion axis), both from
`trackers==2.4.0` (Roboflow), both behind the same `Tracker` contract and with the
same I/O (`detections_to_sv` / `sv_to_tracked`). `trackers` was chosen over
`sv.ByteTrack` because the latter was DEPRECATED in supervision 0.28.0; see
docs/modules/tracking.md.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..config import SiteConfig, TrackerConfig
from ..perception.types import Detection
from .types import TrackedDetection, detections_to_sv, sv_to_tracked


class Tracker(ABC):
    """Tracker contract: detections of a frame -> detections with track_id."""

    @abstractmethod
    def update(self, detections: list[Detection]) -> list[TrackedDetection]:
        """Process a frame. Call ONCE per frame, IN ORDER; state persists."""

    @abstractmethod
    def reset(self) -> "Tracker":
        """Reset the state (IDs included). One session = one reset instance."""


class ByteTracker(Tracker):
    """ByteTrack over `trackers.ByteTrackTracker`.

    Usage requirements (they honor the API): ONE instance per session; `update()`
    once per frame, in order; state persists in the instance. Re-instantiating /
    `reset()` resets the IDs.
    """

    def __init__(
        self, config: TrackerConfig, *, frame_rate: float, lost_track_buffer: int
    ) -> None:
        # `frame_rate` comes from site.fps_ref; `lost_track_buffer` from site.yaml
        # (both site-dependent). The thresholds come from tracker.yaml.
        self._config = config
        self._frame_rate = float(frame_rate)
        self._lost_track_buffer = int(lost_track_buffer)
        self._backend = None
        self.reset()

    def reset(self) -> "ByteTracker":
        from trackers import ByteTrackTracker as _ByteTrackBackend

        self._backend = _ByteTrackBackend(
            lost_track_buffer=self._lost_track_buffer,
            frame_rate=self._frame_rate,
            track_activation_threshold=self._config.track_activation_threshold,
            high_conf_det_threshold=self._config.high_conf_det_threshold,
            minimum_iou_threshold=self._config.minimum_iou_threshold,
            minimum_consecutive_frames=self._config.minimum_consecutive_frames,
        )
        return self

    def update(self, detections: list[Detection]) -> list[TrackedDetection]:
        # Even on frames with no detections, update must be called (with an empty
        # Detections) so that the frame counter advances and lost tracks age
        # correctly.
        sv_dets = detections_to_sv(detections)
        tracked = self._backend.update(sv_dets)
        return sv_to_tracked(tracked)


class OcSortTracker(Tracker):
    """OC-SORT over `trackers.OCSORTTracker` (non-linear motion axis).

    Same contract and same I/O as `ByteTracker`. Its observation-centric mechanism
    (re-update after occlusion via `delta_t` and directional momentum via
    `direction_consistency_weight`) re-associates by the LAST observation +
    direction, instead of the Kalman forward extrapolation — the target of the
    maneuvers during the bus turn. Shared params matched to ByteTrack (clean
    ablation); OC-SORT's own ones at the author defaults.
    """

    def __init__(
        self, config: TrackerConfig, *, frame_rate: float, lost_track_buffer: int
    ) -> None:
        self._config = config
        self._frame_rate = float(frame_rate)
        self._lost_track_buffer = int(lost_track_buffer)
        self._backend = None
        self.reset()

    def reset(self) -> "OcSortTracker":
        from trackers import OCSORTTracker as _OcSortBackend

        self._backend = _OcSortBackend(
            lost_track_buffer=self._lost_track_buffer,
            frame_rate=self._frame_rate,
            high_conf_det_threshold=self._config.high_conf_det_threshold,
            minimum_iou_threshold=self._config.minimum_iou_threshold,
            minimum_consecutive_frames=self._config.minimum_consecutive_frames,
            delta_t=self._config.delta_t,
            direction_consistency_weight=self._config.direction_consistency_weight,
        )
        return self

    def update(self, detections: list[Detection]) -> list[TrackedDetection]:
        return sv_to_tracked(self._backend.update(detections_to_sv(detections)))


# Registry of available implementations, selectable by `tracker_type`.
_TRACKERS: dict[str, type[Tracker]] = {
    "bytetrack": ByteTracker,
    "ocsort": OcSortTracker,
}


def tracker_from_config(
    tracker_config: TrackerConfig, site: SiteConfig, *, override_type: str | None = None
) -> Tracker:
    """Build the tracker indicated by `tracker_type` (or `override_type`).

    `frame_rate` is derived from `site.fps_ref` (not duplicated); `lost_track_buffer`
    from `site.lost_track_buffer_frames` (an occlusion property of the site, shared
    by both trackers). `override_type` allows forcing the tracker from a script
    (for the ablation) without editing the config.
    """
    ttype = (override_type or tracker_config.tracker_type).lower()
    if ttype not in _TRACKERS:
        raise ValueError(
            f"tracker_type desconocido: {ttype!r}. Disponibles: {sorted(_TRACKERS)}"
        )
    return _TRACKERS[ttype](
        tracker_config,
        frame_rate=float(site.fps_ref),
        lost_track_buffer=site.lost_track_buffer_frames,
    )

"""Loading of the tracker configuration (`config/tracker.yaml`).

ONLY site-INDEPENDENT parameters. `tracker_type` selects the algorithm (bytetrack
fixed | ocsort experimental); both share a contract and the `shared:` params
(matched so that the ablation isolates the OC-SORT mechanism). The site-dependent
ones (`lost_track_buffer_frames`, `frame_rate`) live in site.yaml.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from .site import find_project_root


@dataclass(frozen=True)
class ReidSettings:
    """Parameters of the re-id layer (pipeline). Site gates go in SiteConfig."""

    enabled: bool
    model: str  # name of the OSNet weight (swappable piece; resolved in models/)
    device: str
    half: bool
    similarity_threshold: float  # minimum cosine to reconnect (conservative)
    similarity_margin: float  # margin of the best over the 2nd (when in doubt, do not reconnect)
    embedding_refresh_frames: int  # embedding refresh of active tracks (not per-frame)


@dataclass(frozen=True)
class TrackerConfig:
    """Tracker parameters (flat; the factory uses whichever each tracker needs)."""

    tracker_type: str  # "bytetrack" | "ocsort"
    # Shared (same meaning in both trackers).
    high_conf_det_threshold: float
    minimum_iou_threshold: float
    minimum_consecutive_frames: int
    # Specific to ByteTrack.
    track_activation_threshold: float
    # Specific to OC-SORT (its observation-centric mechanism).
    delta_t: int
    direction_consistency_weight: float
    # Re-id layer (Path D).
    reid: ReidSettings


def load_tracker_config(project_root: Optional[Path] = None) -> TrackerConfig:
    """Load and validate `config/tracker.yaml`."""
    root = project_root or find_project_root()
    cfg_path = root / "config" / "tracker.yaml"
    if not cfg_path.is_file():
        raise FileNotFoundError(f"No existe la config del tracker: {cfg_path}")

    with open(cfg_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    shared = data["shared"]
    bytetrack = data["bytetrack"]
    ocsort = data["ocsort"]
    reid = data["reid"]

    return TrackerConfig(
        tracker_type=str(data["tracker_type"]),
        high_conf_det_threshold=float(shared["high_conf_det_threshold"]),
        minimum_iou_threshold=float(shared["minimum_iou_threshold"]),
        minimum_consecutive_frames=int(shared["minimum_consecutive_frames"]),
        track_activation_threshold=float(bytetrack["track_activation_threshold"]),
        delta_t=int(ocsort["delta_t"]),
        direction_consistency_weight=float(ocsort["direction_consistency_weight"]),
        reid=ReidSettings(
            enabled=bool(reid["enabled"]),
            model=str(reid["model"]),
            device=str(reid["device"]),
            half=bool(reid["half"]),
            similarity_threshold=float(reid["similarity_threshold"]),
            similarity_margin=float(reid["similarity_margin"]),
            embedding_refresh_frames=int(reid["embedding_refresh_frames"]),
        ),
    )

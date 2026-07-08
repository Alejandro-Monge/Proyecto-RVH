"""Loading of the detector configuration (`config/detector.yaml`).

Model and inference parameters. They do NOT depend on the site (the model belongs
to the pipeline, not to the intersection), which is why they live apart from
`site.yaml`. The active class filter DOES belong to the site and is read from
`site.yaml` (`class_set`).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from .site import find_project_root


@dataclass(frozen=True)
class DetectorConfig:
    """Model and inference parameters."""

    # Name of the weights file (e.g. "yolov8x.pt"); Ultralytics downloads it if it
    # is not present. Changing it here = changing the model without touching code.
    weights: str
    # Directory (absolute, resolved against the root) where they live/are downloaded.
    weights_dir: Path
    confidence: float  # confidence threshold (PROVISIONAL; sensitivity vs GT later)
    iou: float  # IoU threshold for NMS
    imgsz: int  # side of the inference image
    device: str  # "cuda:0", "cpu", etc.
    half: bool  # FP16 (real-time deployment / Jetson)
    # INTER-CLASS NMS: suppresses overlapping boxes from DIFFERENT CLASSES (a large vehicle
    # detected as bus+truck). Intra-class NMS (default) lets them through → concurrent double id
    # (front b, cross-class). Default False for backward compatibility; enabled by config.
    agnostic_nms: bool = False

    @property
    def weights_path(self) -> Path:
        """Full path to the weights file."""
        return self.weights_dir / self.weights


def load_detector_config(project_root: Optional[Path] = None) -> DetectorConfig:
    """Load and validate `config/detector.yaml`."""
    root = project_root or find_project_root()
    cfg_path = root / "config" / "detector.yaml"
    if not cfg_path.is_file():
        raise FileNotFoundError(f"No existe la config del detector: {cfg_path}")

    with open(cfg_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    model = data["model"]
    inf = data["inference"]
    weights_dir = (root / model["weights_dir"]).resolve()

    return DetectorConfig(
        weights=str(model["weights"]),
        weights_dir=weights_dir,
        confidence=float(inf["confidence"]),
        iou=float(inf["iou"]),
        imgsz=int(inf["imgsz"]),
        device=str(inf["device"]),
        half=bool(inf["half"]),
        agnostic_nms=bool(inf.get("agnostic_nms", False)),
    )

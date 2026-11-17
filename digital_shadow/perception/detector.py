"""Vehicle detector: contract and YOLO implementation (Ultralytics).

The detector is a CONSUMER of the video source. Its central operation is pure:
`detect(image) -> list[Detection]` (image in, detections out). It knows NOTHING about
pacing or discard (the source governs that), nor about the timestamp (the driver matches
it). The model is a config parameter: switching to a lighter one does not touch this code.
"""

from __future__ import annotations

import logging
import shutil
from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np

from ..config import DetectorConfig
from .coco import names_to_ids
from .types import Detection

logger = logging.getLogger(__name__)


def _to_numpy(x) -> np.ndarray:
    """Convert a tensor (torch) or array to numpy, without assuming a backend."""
    if hasattr(x, "cpu"):
        x = x.cpu()
    if hasattr(x, "numpy"):
        return x.numpy()
    return np.asarray(x)


class Detector(ABC):
    """Detector contract: image → list of detections (raw corners)."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection]:
        """Detect in a BGR frame. Returns boxes in image px."""

    @property
    @abstractmethod
    def class_ids(self) -> list[int]:
        """Active COCO ids (the site filter translated to ids)."""


class YoloDetector(Detector):
    """Detector based on `ultralytics.YOLO` with pretrained COCO weights.

    Filters the inference to the site's `class_set` (translated to COCO ids from the
    model itself, with no hardcoded ids), so that YOLO does not waste effort on classes
    that are not of interest.
    """

    def __init__(self, config: DetectorConfig, class_names: list[str]) -> None:
        # `class_names` = active filter of the site (site.yaml `class_set`).
        self._cfg = config
        self._class_names = list(class_names)
        self._model = None
        self._model_names: dict[int, str] = {}
        self._class_ids: list[int] = []

    def load(self) -> "YoloDetector":
        """Load the model and resolve the class filter. Downloads weights if missing."""
        from ultralytics import YOLO

        weights_path = self._resolve_weights()
        self._model = YOLO(str(weights_path))
        # `names` is the id→name map the model brings (authoritative).
        self._model_names = dict(self._model.names)
        self._class_ids = names_to_ids(self._class_names, self._model_names)
        logger.info(
            "Modelo cargado: %s | clases activas %s -> ids %s",
            weights_path.name,
            self._class_names,
            self._class_ids,
        )
        return self

    def _resolve_weights(self) -> Path:
        """Return the path to the weights in `models/`, downloading them if missing.

        Ultralytics downloads a known asset (yolov8x.pt) by name to the CWD; if that
        happens, we move the file to `weights_dir` to fix its location and make the
        download idempotent (reproducibility).
        """
        weights_path = self._cfg.weights_path
        weights_path.parent.mkdir(parents=True, exist_ok=True)
        if weights_path.exists():
            return weights_path

        from ultralytics.utils.downloads import attempt_download_asset

        # Download the asset (to CWD by name) and normalize it to weights_dir.
        downloaded = Path(attempt_download_asset(self._cfg.weights))
        if downloaded.resolve() != weights_path.resolve():
            shutil.move(str(downloaded), str(weights_path))
        return weights_path

    def detect(self, image: np.ndarray) -> list[Detection]:
        if self._model is None:
            raise RuntimeError("detect() llamado antes de load()")
        results = self._model.predict(
            image,
            conf=self._cfg.confidence,
            iou=self._cfg.iou,
            imgsz=self._cfg.imgsz,
            device="cpu",
            half=False,
            classes=self._class_ids,
            agnostic_nms=self._cfg.agnostic_nms,   # inter-class NMS (front b, cross-class)
            verbose=False,
        )
        return self._parse(results[0])

    def warmup(self, height: int, width: int) -> None:
        """A dummy inference to pay the CUDA init before measuring/opening."""
        dummy = np.zeros((height, width, 3), dtype=np.uint8)
        self.detect(dummy)

    def _parse(self, result) -> list[Detection]:
        """Convert an Ultralytics `Results` into `list[Detection]`."""
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = _to_numpy(boxes.xyxy)
        conf = _to_numpy(boxes.conf)
        cls = _to_numpy(boxes.cls)
        detections: list[Detection] = []
        for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
            cid = int(k)
            detections.append(
                Detection(
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                    confidence=float(c),
                    class_id=cid,
                    class_name=self._model_names.get(cid, str(cid)),
                )
            )
        return detections

    @property
    def class_ids(self) -> list[int]:
        return list(self._class_ids)

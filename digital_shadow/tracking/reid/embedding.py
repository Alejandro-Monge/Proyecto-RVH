"""Seam of the appearance embedding model (re-id).

`EmbeddingModel` is the SWAPPABLE piece: today generic OSNet (via boxmot), as a MEASUREMENT
of the floor of the benefit; tomorrow a custom model fine-tuned to the site domain, changing
ONLY this piece (not the reconnector nor the tracker). The extractor receives boxes + the
(clean) frame and returns one vector per box; the crop and the preprocessing live INSIDE the
model (each model wants its own).
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


class EmbeddingModel(ABC):
    """Appearance extractor: boxes + frame -> one embedding per box."""

    @abstractmethod
    def extract(self, boxes_xyxy: np.ndarray, frame: np.ndarray) -> np.ndarray:
        """Returns `(N, D)` float32: one embedding per box `(N,4)` cropped from the frame."""


class OsnetEmbeddingModel(EmbeddingModel):
    """Generic pretrained OSNet, via boxmot's re-id extractor.

    A GENERIC model (person re-id) used as a floor measurement: if it reconnects little in a
    top-down scene of nearly identical vehicles, the bottleneck is the model (→ justifies the
    custom one), not the Path D architecture. boxmot is isolated behind this class; tomorrow's
    custom model does not drag it along.
    """

    def __init__(self, weights_path: Path, device: str = "cuda:0", half: bool = True) -> None:
        self._weights_path = Path(weights_path)
        self._device = device
        self._half = half
        self._reid = None

    def load(self) -> "OsnetEmbeddingModel":
        from boxmot.reid.core.reid import ReID

        # boxmot downloads the weight to the given absolute path if missing (idempotent).
        self._weights_path.parent.mkdir(parents=True, exist_ok=True)
        self._reid = ReID(
            weights=self._weights_path.resolve(), device=self._device, half=self._half
        )
        logger.info("OSNet cargado: %s (device=%s, half=%s)",
                    self._weights_path.name, self._device, self._half)
        return self

    def extract(self, boxes_xyxy: np.ndarray, frame: np.ndarray) -> np.ndarray:
        if self._reid is None:
            raise RuntimeError("extract() llamado antes de load()")
        boxes = np.asarray(boxes_xyxy, dtype=float)
        if boxes.shape[0] == 0:
            return np.empty((0, 512), dtype=np.float32)
        feats = self._reid.model.get_features(boxes, frame)
        return np.asarray(feats, dtype=np.float32)


class CustomEmbeddingModel(EmbeddingModel):
    """Placeholder for a CUSTOM fine-tuned embedding model (site domain).

    Marks the seam: implement `load()` (load your own checkpoint) and `extract()` (crop +
    preprocess + forward). Changing `reid.model` in config to this model does NOT require
    touching the reconnector nor the tracker. (Not implemented: it will be trained and wired
    when the floor measurement with generic OSNet justifies it.)
    """

    def extract(self, boxes_xyxy: np.ndarray, frame: np.ndarray) -> np.ndarray:
        raise NotImplementedError(
            "CustomEmbeddingModel es el hueco para el modelo fine-tuneado; aún no existe."
        )

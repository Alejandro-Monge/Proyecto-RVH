"""Appearance-based tracklet reconnection (layer after ByteTrack).

ByteTrack is NOT touched: when it fragments (an occluded vehicle reappears with a new ID),
this layer tries to inherit the old ID to the new one if the APPEARANCE matches, with
spatiotemporal gating (it only compares against the few lost tracks near in time and space)
and a conservative threshold + margin (when in doubt, do NOT reconnect — a swap is worse than
a fragmentation for the digital shadow). The embeddings are extracted from CLEAN frames (birth
+ refresh while the track is visible), never from the occluded object, and only when needed
(not per-frame).

With re-id disabled, `NoOpReconnector` is used, a BIT-IDENTICAL passthrough to the base
ByteTrack (guarantees a clean ablation). Identical output contract (`list[TrackedDetection]`
with reassigned `track_id`) → the downstream stage does not change.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from ...config import ReidSettings, SiteConfig
from ..types import TrackedDetection
from .embedding import EmbeddingModel


class Reconnector(ABC):
    """Layer that reassigns IDs of fragmented tracklets (or passthrough)."""

    @abstractmethod
    def apply(
        self, tracked: list[TrackedDetection], frame: np.ndarray, frame_index: int
    ) -> list[TrackedDetection]:
        """Returns the tracks with (possibly) reassigned IDs."""


class NoOpReconnector(Reconnector):
    """BIT-IDENTICAL passthrough: re-id disabled. It does not instantiate a model nor touch anything."""

    def apply(self, tracked, frame, frame_index):  # noqa: D102
        return tracked


@dataclass
class _Seen:
    last_frame: int
    box: np.ndarray  # xyxy of the last appearance
    emb: np.ndarray | None  # normalized embedding (or None if not yet extracted)
    emb_frame: int  # frame of the last embedding


def _center(box: np.ndarray) -> np.ndarray:
    return np.array([(box[0] + box[2]) * 0.5, (box[1] + box[3]) * 0.5])


def _normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / (n + 1e-9)


class ReidReconnector(Reconnector):
    """Appearance-based reconnection with spatiotemporal gating and a conservative threshold."""

    def __init__(
        self,
        embedding_model: EmbeddingModel,
        *,
        similarity_threshold: float,
        similarity_margin: float,
        embedding_refresh_frames: int,
        temporal_window_frames: int,
        spatial_corridor_px: float,
    ) -> None:
        self._model = embedding_model
        self._sim_thr = similarity_threshold
        self._margin = similarity_margin
        self._refresh = embedding_refresh_frames
        self._temporal_window = temporal_window_frames
        self._corridor = spatial_corridor_px
        self._seen: dict[int, _Seen] = {}     # bt_id -> last info + embedding
        self._remap: dict[int, int] = {}       # bt_id -> inherited canonical id
        self._consumed: set[int] = set()        # ids already inherited (do not re-match)

    def _canonical(self, tid: int) -> int:
        seen = set()
        while tid in self._remap and tid not in seen:
            seen.add(tid)
            tid = self._remap[tid]
        return tid

    def apply(self, tracked, frame, frame_index):
        current = {td.track_id: td for td in tracked}

        # 1) Which tracks need an embedding this frame? New ones (for matching) and
        #    active ones whose embedding is due for a refresh. Batch extraction (1 call).
        to_embed = []  # (bt_id, box)
        new_ids = []
        for tid, td in current.items():
            box = np.array([td.x1, td.y1, td.x2, td.y2])
            is_new = tid not in self._seen and tid not in self._remap
            if is_new:
                new_ids.append(tid)
                to_embed.append((tid, box))
            elif tid in self._seen and frame_index - self._seen[tid].emb_frame >= self._refresh:
                to_embed.append((tid, box))

        embeddings: dict[int, np.ndarray] = {}
        if to_embed:
            boxes = np.stack([b for _, b in to_embed])
            feats = _normalize(self._model.extract(boxes, frame))
            for (tid, _), f in zip(to_embed, feats):
                embeddings[tid] = f

        # 2) Try to reconnect each NEW track against the gated lost ones.
        for tid in new_ids:
            td = current[tid]
            new_box = np.array([td.x1, td.y1, td.x2, td.y2])
            new_emb = embeddings.get(tid)
            if new_emb is None:
                continue
            cand = self._gated_candidates(new_box, frame_index, exclude=current.keys())
            best_id, best_sim, second_sim = None, -1.0, -1.0
            for cid in cand:
                sim = float(self._seen[cid].emb @ new_emb)
                if sim > best_sim:
                    best_id, second_sim, best_sim = cid, best_sim, sim
                elif sim > second_sim:
                    second_sim = sim
            # Conservative threshold + margin: when in doubt (a tie), do NOT reconnect.
            if best_id is not None and best_sim >= self._sim_thr and (
                best_sim - second_sim
            ) >= self._margin:
                self._remap[tid] = self._canonical(best_id)
                self._consumed.add(best_id)

        # 3) Update the seen registry (position + refreshed embedding).
        for tid, td in current.items():
            box = np.array([td.x1, td.y1, td.x2, td.y2])
            emb = embeddings.get(tid)
            if tid in self._seen:
                self._seen[tid].last_frame = frame_index
                self._seen[tid].box = box
                if emb is not None:
                    self._seen[tid].emb = emb
                    self._seen[tid].emb_frame = frame_index
            else:
                self._seen[tid] = _Seen(frame_index, box, emb, frame_index)

        # 4) Emit with the canonical ID (includes the ones reassigned this frame).
        out = []
        for td in tracked:
            cid = self._canonical(td.track_id)
            out.append(td if cid == td.track_id else _with_id(td, cid))
        return out

    def _gated_candidates(self, new_box, frame_index, exclude) -> list[int]:
        """Lost ones within the temporal window and the spatial corridor, with an embedding."""
        c_new = _center(new_box)
        out = []
        for cid, s in self._seen.items():
            if cid in exclude or cid in self._consumed or s.emb is None:
                continue
            dt = frame_index - s.last_frame
            if not (0 < dt <= self._temporal_window):
                continue
            if np.linalg.norm(_center(s.box) - c_new) > self._corridor:
                continue
            out.append(cid)
        return out


def _with_id(td: TrackedDetection, new_id: int) -> TrackedDetection:
    return TrackedDetection(new_id, td.x1, td.y1, td.x2, td.y2, td.class_id, td.confidence)


def reconnector_from_config(tracker_config, site: SiteConfig) -> Reconnector:
    """Build the re-id layer according to config. enabled:false -> exact passthrough."""
    reid: ReidSettings = tracker_config.reid
    if not reid.enabled:
        return NoOpReconnector()

    from ...config import find_project_root
    from .embedding import OsnetEmbeddingModel

    weights = find_project_root() / "models" / f"{reid.model}.pt"
    model = OsnetEmbeddingModel(weights, device=reid.device, half=reid.half).load()
    return ReidReconnector(
        model,
        similarity_threshold=reid.similarity_threshold,
        similarity_margin=reid.similarity_margin,
        embedding_refresh_frames=reid.embedding_refresh_frames,
        temporal_window_frames=site.reid_temporal_window_frames,
        spatial_corridor_px=site.reid_spatial_corridor_px,
    )

"""Deterministic tests of the re-id layer (no OSNet/GPU).

The embedding model is injectable (`FakeEmbeddingModel`, which returns controlled
vectors), as we did with Clock/FrameReader. This verifies the reconnector's LOGIC
—temporal/spatial gating, threshold, margin, remap persistence, exact
passthrough— in milliseconds, without the real OSNet inference (which is
exercised in the runs). Runnable with pytest or as a script.
"""

from __future__ import annotations

import numpy as np

from digital_shadow.tracking import NoOpReconnector
from digital_shadow.tracking.reid.embedding import EmbeddingModel
from digital_shadow.tracking.reid.reconnector import ReidReconnector
from digital_shadow.tracking.types import TrackedDetection


class FakeEmbeddingModel(EmbeddingModel):
    """Returns a fixed embedding per 'appearance identity' according to a marker.

    The identity is inferred from the box's x1 coordinate (a controllable proxy in
    the test): boxes with the same `appearance_key` give the same vector → cosine 1.0.
    """

    def __init__(self, key_to_vec: dict[int, np.ndarray]) -> None:
        self._map = key_to_vec

    def extract(self, boxes_xyxy: np.ndarray, frame: np.ndarray) -> np.ndarray:
        out = []
        for b in np.asarray(boxes_xyxy, dtype=float):
            key = int(round(b[0]))  # x1 as appearance key (test proxy)
            out.append(self._map[key])
        return np.asarray(out, dtype=np.float32)


# Two orthogonal appearances (cosine 0 with each other, 1 with themselves).
APP_A = np.array([1.0, 0.0, 0.0], dtype=np.float32)
APP_B = np.array([0.0, 1.0, 0.0], dtype=np.float32)


def _td(track_id, x1, y1=200.0, app=None):
    # x1 encodes both position and (via FakeEmbeddingModel) appearance.
    return TrackedDetection(track_id, x1, y1, x1 + 40, y1 + 60, 2, 0.9)


def _reconnector(model, *, thr=0.6, margin=0.05, window=140, corridor=250.0):
    return ReidReconnector(
        model, similarity_threshold=thr, similarity_margin=margin,
        embedding_refresh_frames=15, temporal_window_frames=window,
        spatial_corridor_px=corridor,
    )


def test_noop_passthrough_identical() -> None:
    rc = NoOpReconnector()
    tracks = [_td(5, 100), _td(6, 300)]
    out = rc.apply(tracks, frame=None, frame_index=0)
    assert out is tracks  # exact passthrough, no copy, no changes


def test_reconnects_same_appearance_within_gates() -> None:
    # Track 1 (appearance A) visible, disappears, and track 2 is born (same appearance A,
    # near in space/time) -> must inherit the canonical id 1.
    model = FakeEmbeddingModel({100: APP_A, 130: APP_A})
    rc = _reconnector(model)
    for f in range(5):
        rc.apply([_td(1, 100)], frame=None, frame_index=f)
    # gap (track 1 disappears) and reappearance as track 2, 30 px to the right
    out = rc.apply([_td(2, 130)], frame=None, frame_index=20)
    assert out[0].track_id == 1  # reconnected


def test_no_reconnect_out_of_temporal_window() -> None:
    model = FakeEmbeddingModel({100: APP_A, 130: APP_A})
    rc = _reconnector(model, window=10)
    for f in range(5):
        rc.apply([_td(1, 100)], frame=None, frame_index=f)
    out = rc.apply([_td(2, 130)], frame=None, frame_index=100)  # dt=96 > 10
    assert out[0].track_id == 2  # does NOT reconnect (out of window)


def test_no_reconnect_out_of_spatial_corridor() -> None:
    model = FakeEmbeddingModel({100: APP_A, 900: APP_A})
    rc = _reconnector(model, corridor=50.0)
    for f in range(5):
        rc.apply([_td(1, 100)], frame=None, frame_index=f)
    out = rc.apply([_td(2, 900)], frame=None, frame_index=20)  # far
    assert out[0].track_id == 2  # does NOT reconnect (out of corridor)


def test_no_reconnect_below_threshold() -> None:
    # Different appearance (B) -> cosine 0 < threshold -> does not reconnect.
    model = FakeEmbeddingModel({100: APP_A, 130: APP_B})
    rc = _reconnector(model)
    for f in range(5):
        rc.apply([_td(1, 100)], frame=None, frame_index=f)
    out = rc.apply([_td(2, 130)], frame=None, frame_index=20)
    assert out[0].track_id == 2  # does NOT reconnect (different appearance)


def test_no_reconnect_on_ambiguous_tie() -> None:
    # Two lost candidates with the SAME appearance A nearby -> tie -> do NOT reconnect
    # (margin): an ID swap is worse than a fragmentation.
    model = FakeEmbeddingModel({100: APP_A, 150: APP_A, 130: APP_A})
    rc = _reconnector(model, margin=0.05)
    for f in range(5):
        rc.apply([_td(1, 100), _td(2, 150)], frame=None, frame_index=f)
    # track 3 is born (appearance A) equidistant -> two candidates tied at cosine 1.0
    out = rc.apply([_td(3, 130)], frame=None, frame_index=20)
    assert out[0].track_id == 3  # does NOT reconnect (tie within the margin)


def test_remap_persists_across_frames() -> None:
    model = FakeEmbeddingModel({100: APP_A, 130: APP_A})
    rc = _reconnector(model)
    for f in range(5):
        rc.apply([_td(1, 100)], frame=None, frame_index=f)
    rc.apply([_td(2, 130)], frame=None, frame_index=20)  # reconnects 2->1
    # in later frames, track 2 keeps being emitted as 1
    out = rc.apply([_td(2, 132)], frame=None, frame_index=21)
    assert out[0].track_id == 1


def _run_all() -> None:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"OK  {fn.__name__}")
    print(f"\n{len(tests)} tests deterministas de re-id pasaron.")


if __name__ == "__main__":
    _run_all()

"""Deterministic tests for the clock and frame discarding of `FileVideoSource`.

They verify in milliseconds —without tying to a 180 s run and in a REPEATABLE
way (always the same discard pattern)— the properties that a test over the real
file cannot demonstrate rigorously:

  * the invariant `delivered + discarded == total` (the discard rate would be
    a lie if it did not hold);
  * the EXACT count of discarded frames against a simulated slow consumer;
  * EOF detection on the correct frame (last index of the media).

This is achieved by injecting a fake clock (controls 'what time it is' without
waiting) and a fake frame reader (N synthetic frames; each frame encodes its
index to assert which one was delivered). Runnable with pytest or as a script
(`python -m tests.test_file_source`).
"""

from __future__ import annotations

from fractions import Fraction

import numpy as np

from digital_shadow.sources.base import Clock, FrameReader
from digital_shadow.sources.file_source import FileVideoSource

FPS = Fraction(10)  # period = 0.1 s; values chosen for exact float arithmetic


# --------------------------------------------------------------------------- #
# Test doubles
# --------------------------------------------------------------------------- #
class FakeClock(Clock):
    """Controllable clock: `now` returns the set time; `sleep`/`advance` move it."""

    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            self.t += seconds

    def advance(self, seconds: float) -> None:
        """Simulates that the consumer took `seconds` to process (its 'detection')."""
        self.t += seconds


class FakeFrameReader(FrameReader):
    """Synthetic reader of N frames. Each frame is an array that encodes its index."""

    def __init__(self, n_frames: int, resolution: tuple[int, int] = (4, 4)) -> None:
        self.n = n_frames
        self._res = resolution
        self._pos = -1  # index of the last frame advanced by grab()
        self.released = False

    def grab(self) -> bool:
        if self._pos + 1 >= self.n:
            return False
        self._pos += 1
        return True

    def retrieve(self) -> np.ndarray:
        return np.full((1, 1), self._pos, dtype=np.int32)

    def resolution(self) -> tuple[int, int]:
        return self._res

    def release(self) -> None:
        self.released = True


def _make_source(n_frames: int, clock: FakeClock, fps: Fraction = FPS) -> FileVideoSource:
    reader = FakeFrameReader(n_frames)
    src = FileVideoSource("fake.mp4", fps, clock=clock, frame_reader=reader)
    src.open()
    return src


def _drain(src: FileVideoSource, clock: FakeClock, work_s: float) -> list[int]:
    """Consumes until EOF advancing the clock by `work_s` per frame. Returns indices."""
    delivered: list[int] = []
    while True:
        frame = src.read()
        if frame.end_of_source:
            break
        delivered.append(int(frame.image[0, 0]))
        clock.advance(work_s)
    return delivered


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #
def test_slow_consumer_exact_discard_and_invariant() -> None:
    # Consumer at 0.25 s/frame (= 4 fps), slower than the source's 10 fps:
    # it must discard and the pattern is deterministic.
    clock = FakeClock(0.0)
    src = _make_source(20, clock)
    delivered = _drain(src, clock, work_s=0.25)

    # Pattern computed by hand: these indices are delivered.
    assert delivered == [0, 2, 5, 7, 10, 12, 15, 17, 19]
    assert src.delivered_frames == 9
    assert src.discarded_frames == 11
    # The first number: the invariant. It must give EXACTLY the total.
    assert src.delivered_frames + src.discarded_frames == 20
    # EOF on the last index of the media.
    assert delivered[-1] == 19


def test_fast_consumer_waits_no_discard() -> None:
    # Instantaneous consumer (0 s/frame): the source waits, discards nothing and
    # delivers ALL frames in order.
    clock = FakeClock(0.0)
    src = _make_source(20, clock)
    delivered = _drain(src, clock, work_s=0.0)

    assert delivered == list(range(20))
    assert src.delivered_frames == 20
    assert src.discarded_frames == 0
    assert src.delivered_frames + src.discarded_frames == 20


def test_eof_on_single_huge_jump() -> None:
    # A huge clock jump after the first frame: almost everything is discarded, the
    # last available frame is delivered and then EOF. The invariant holds.
    clock = FakeClock(0.0)
    src = _make_source(20, clock)

    first = src.read()
    assert int(first.image[0, 0]) == 0
    clock.advance(100.0)
    second = src.read()
    assert int(second.image[0, 0]) == 19  # last index, not one out of range

    third = src.read()
    assert third.end_of_source is True
    assert src.delivered_frames == 2          # frames 0 and 19
    assert src.discarded_frames == 18         # frames 1..18
    assert src.delivered_frames + src.discarded_frames == 20


def test_end_of_source_is_terminal_and_idempotent() -> None:
    clock = FakeClock(0.0)
    src = _make_source(5, clock)
    _drain(src, clock, work_s=0.0)
    delivered_at_eof = src.delivered_frames
    discarded_at_eof = src.discarded_frames

    # Re-reading after EOF keeps returning end_of_source and does NOT alter the counters.
    for _ in range(3):
        frame = src.read()
        assert frame.end_of_source is True
        assert frame.image is None
    assert src.delivered_frames == delivered_at_eof
    assert src.discarded_frames == discarded_at_eof


def test_empty_source_terminates_immediately() -> None:
    clock = FakeClock(0.0)
    src = _make_source(0, clock)
    frame = src.read()
    assert frame.end_of_source is True
    assert src.delivered_frames == 0
    assert src.discarded_frames == 0


def test_timestamp_is_media_time() -> None:
    # timestamp_s of the delivered frame = index / fps (media time, not wall).
    clock = FakeClock(0.0)
    fps = Fraction(355, 12)  # the real fps of the Rusia site
    reader = FakeFrameReader(50)
    src = FileVideoSource("fake.mp4", fps, clock=clock, frame_reader=reader)
    src.open()

    frame0 = src.read()
    assert frame0.timestamp_s == 0.0
    clock.advance(1.0)  # 1 s of clock -> expected index floor(1*355/12)=29
    frame1 = src.read()
    assert int(frame1.image[0, 0]) == 29
    assert frame1.timestamp_s == float(Fraction(29, 1) / fps)  # 29*12/355


def test_read_before_open_raises() -> None:
    reader = FakeFrameReader(5)
    src = FileVideoSource("fake.mp4", FPS, clock=FakeClock(0.0), frame_reader=reader)
    try:
        src.read()
    except RuntimeError:
        pass
    else:
        raise AssertionError("read() antes de open() debió lanzar RuntimeError")


def test_close_releases_reader_and_terminates() -> None:
    clock = FakeClock(0.0)
    reader = FakeFrameReader(10)
    src = FileVideoSource("fake.mp4", FPS, clock=clock, frame_reader=reader)
    src.open()
    src.read()
    src.close()
    assert reader.released is True
    assert src.read().end_of_source is True


def _run_all() -> None:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"OK  {fn.__name__}")
    print(f"\n{len(tests)} tests deterministas pasaron.")


if __name__ == "__main__":
    _run_all()

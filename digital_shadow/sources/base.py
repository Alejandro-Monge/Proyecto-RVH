"""Video source contract and its injectable dependencies.

Here lives the design BOUNDARY: the pipeline (perception, above) only knows
`VideoSource` and `Frame`. Switching prerecorded file ↔ live camera means replacing
the `VideoSource` implementation without touching anything above this boundary
(non-negotiable requirement of `docs/design_video_source_interface.md` Section 1,
Section 5).

The two dependencies the source needs — the CLOCK and the FRAME READER — are declared
here as injectable abstractions. In production the real implementations are used
(`MonotonicClock`, `OpenCVFrameReader`); in test, controllable doubles are injected to
verify the clock/discard deterministically and in milliseconds, without tying to a
180 s run.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import NamedTuple, Optional

import numpy as np


class Frame(NamedTuple):
    """The only thing that crosses the boundary: a timestamped frame.

    It is a tuple (it satisfies the literal contract `(frame, timestamp, end)`) with
    readable names. It carries NO resolution, fps, or any other property of the media:
    those live in the site config and the pipeline reads them from there, not from the
    source (so no property of the concrete video leaks into the pipeline).
    """

    # BGR image (numpy.ndarray) of the delivered frame, or None when `end_of_source`
    # is True (there is no image to deliver).
    image: Optional[np.ndarray]
    # Seconds since the session t0. In file mode = media time of the delivered frame
    # (= n_frame / fps_ref): its real index, NOT wall-clock. This way it stays
    # comparable with the ground truth regardless of which frames were discarded
    # (design Section 3.1).
    timestamp_s: float
    # Explicit, terminal signal that there are no more frames (end of media or closed
    # source). Once True, subsequent reads keep returning True.
    end_of_source: bool


# --------------------------------------------------------------------------- #
# Injectable clock
# --------------------------------------------------------------------------- #
class Clock(ABC):
    """Time source. Abstracts 'what time is it' and 'wait'.

    Injectable so that the pacing/discard — the heart of the module — is verifiable
    deterministically: a fake clock controls time without really waiting, reproducing
    the same discard pattern as many times as needed (which a run tied to the real
    clock does NOT allow).
    """

    @abstractmethod
    def now(self) -> float:
        """Current instant in seconds (arbitrary but monotonic origin)."""

    @abstractmethod
    def sleep(self, seconds: float) -> None:
        """Block for `seconds` seconds. If `seconds <= 0`, does nothing."""


class MonotonicClock(Clock):
    """Production clock: `time.monotonic` (immune to wall-clock jumps).

    It is not hard real time: the scheduler granularity (~1 ms) is negligible against
    the frame period (12/355 ≈ 33.8 ms), but it is documented as a limit.
    """

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


# --------------------------------------------------------------------------- #
# Injectable frame reader
# --------------------------------------------------------------------------- #
class FrameReader(ABC):
    """Low-level sequential access to the media: advance and decode.

    Separates `grab` (advance to the next frame WITHOUT decoding, cheap) from
    `retrieve` (decode the last advanced frame). That separation is what makes discard
    cheap: to skip frames, `grab` alone suffices without paying the decoding.
    Injectable so the test uses a synthetic reader of N frames and verifies the EOF
    edge without the real file.
    """

    @abstractmethod
    def grab(self) -> bool:
        """Advance to the next frame without decoding. False upon reaching the end."""

    @abstractmethod
    def retrieve(self) -> np.ndarray:
        """Decode and return the last frame advanced by `grab`."""

    @abstractmethod
    def resolution(self) -> tuple[int, int]:
        """Media resolution as (width, height) in pixels."""

    @abstractmethod
    def release(self) -> None:
        """Release the media resources."""


# --------------------------------------------------------------------------- #
# Source contract
# --------------------------------------------------------------------------- #
class VideoSource(ABC):
    """Real-time video source interface. The design boundary.

    Three operations (design Section 3): `open` (fixes the session t0), `read`
    (delivers the frame of the current moment), and `close`. The source also carries
    the COUNTER of discarded frames (design Section 4), a first-order performance
    metric.

    Supports `with` to guarantee `open`/`close` even in the face of exceptions.
    """

    @abstractmethod
    def open(self) -> None:
        """Leaves the source ready to deliver frames and fixes the session t0."""

    @abstractmethod
    def read(self) -> Frame:
        """Returns the frame corresponding to the current clock instant."""

    @abstractmethod
    def close(self) -> None:
        """Releases the source. After closing, `read` returns `end_of_source`."""

    @property
    @abstractmethod
    def delivered_frames(self) -> int:
        """Frames delivered to the consumer in the session."""

    @property
    @abstractmethod
    def discarded_frames(self) -> int:
        """Frames discarded by the interface in the session (denominator: total)."""

    def __enter__(self) -> "VideoSource":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

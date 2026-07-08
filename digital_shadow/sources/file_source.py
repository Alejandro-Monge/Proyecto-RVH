"""Video source from file, pretending to be a camera (real-time mode).

Implements `VideoSource` for a prerecorded file. There is a SINGLE mode: real-time
(design Section 2). The file delivers frames at the pace of the site's `fps_ref` and
DISCARDS the ones the consumer does not request in time, exactly like a camera that
waits for no one. All the clock/discard logic — the heart of the module — lives here;
it relies on injectable `Clock` and `FrameReader` (see `base.py`).
"""

from __future__ import annotations

import logging
import math
from fractions import Fraction
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from .base import Clock, Frame, FrameReader, MonotonicClock, VideoSource

logger = logging.getLogger(__name__)


class OpenCVFrameReader(FrameReader):
    """Production `FrameReader` over `cv2.VideoCapture`.

    Uses grab/retrieve separately: the sequential frame-by-frame advance is EXACT
    (unlike `CAP_PROP_POS_FRAMES`, which jumps to the nearest keyframe in codecs with
    inter-frame compression and returns the wrong frame), and faithfully models a
    camera: frames flow forward, never backward.
    """

    def __init__(self, path: Path) -> None:
        self._cap = cv2.VideoCapture(str(path))
        if not self._cap.isOpened():
            raise RuntimeError(f"No se pudo abrir el video: {path}")

    def grab(self) -> bool:
        return self._cap.grab()

    def retrieve(self) -> np.ndarray:
        ok, frame = self._cap.retrieve()
        if not ok:
            # Should not happen: it is only called after a successful grab().
            raise RuntimeError("retrieve() falló tras un grab() exitoso")
        return frame

    def resolution(self) -> tuple[int, int]:
        width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return (width, height)

    def release(self) -> None:
        self._cap.release()


class FileVideoSource(VideoSource):
    """Prerecorded file served in real-time mode, with discard.

    The media instant of frame `k` is `k / fps_ref`. In `open`, `t0 = clock.now()` is
    fixed ('the camera turns on'). On each `read`, according to the elapsed clock time,
    the frame corresponding to 'now' is delivered and the intermediate ones the
    consumer did not manage to request are discarded.

    Guaranteed invariant: upon signaling `end_of_source`,
    `delivered_frames + discarded_frames == total frames of the segment`, that is, each
    media index was delivered or discarded exactly once. This is what makes the discard
    rate (discarded / total) honest.
    """

    def __init__(
        self,
        video_path: Path,
        fps_ref: Fraction,
        *,
        clock: Optional[Clock] = None,
        frame_reader: Optional[FrameReader] = None,
        expected_resolution: Optional[tuple[int, int]] = None,
    ) -> None:
        # `frame_reader` and `clock` are injected in test; in production the real ones
        # are used. `expected_resolution` (from config) only serves for a sanity warning
        # on open; it is NOT exposed through the contract.
        self._video_path = Path(video_path)
        self._fps = Fraction(fps_ref)
        self._clock: Clock = clock if clock is not None else MonotonicClock()
        self._injected_reader = frame_reader
        self._expected_resolution = expected_resolution

        self._reader: Optional[FrameReader] = None
        self._opened = False

        # Session t0 (clock instant at `open`).
        self._t0 = 0.0
        # Index of the last frame DELIVERED to the consumer (-1 = none yet).
        self._delivered_index = -1
        # Index of the last frame ADVANCED by the decoder (-1 = none yet).
        self._decoded_index = -1
        # Session counters.
        self._delivered = 0
        self._discarded = 0
        # The decoder reached the physical end of the media (grab() returned False).
        self._eof_reached = False
        # end_of_source has already been signaled to the consumer (terminal state).
        self._terminated = False
        # Last delivered timestamp (repeated when signaling end_of_source).
        self._last_ts = 0.0

    # -- life cycle -------------------------------------------------------- #
    def open(self) -> None:
        if self._opened:
            return
        self._reader = (
            self._injected_reader
            if self._injected_reader is not None
            else OpenCVFrameReader(self._video_path)
        )
        # Sanity warning: the real resolution should match the one in config. It is
        # not corrected or aborted: it is only reported (the homography is calibrated
        # on the image as is; on the SAR/non-square pixel).
        if self._expected_resolution is not None:
            actual = self._reader.resolution()
            if actual != self._expected_resolution:
                logger.warning(
                    "Resolución real %s != config %s (%s)",
                    actual,
                    self._expected_resolution,
                    self._video_path,
                )
        # t0 on open: the camera has been running since it was turned on. If more than
        # one period passes between open and the first read, those first frames are
        # discarded; it is the correct behavior of a camera.
        self._t0 = self._clock.now()
        self._opened = True

    def close(self) -> None:
        if self._reader is not None:
            self._reader.release()
            self._reader = None
        self._terminated = True
        self._opened = False

    # -- reading ----------------------------------------------------------- #
    def read(self) -> Frame:
        if not self._opened and not self._terminated:
            raise RuntimeError("read() llamado antes de open()")
        if self._terminated:
            return Frame(None, self._last_ts, True)

        fps = self._fps
        next_index = self._delivered_index + 1

        # 1) Which frame corresponds to 'now'? The highest-index one whose media
        #    instant (k / fps) has already passed: floor(t_now * fps).
        t_now = self._clock.now() - self._t0
        n_due = math.floor(t_now * fps)

        if n_due < next_index:
            # Consumer AHEAD: the next frame is not due yet. Wait until its exact
            # instant and deliver it (without duplicating, without busy-spin).
            target = next_index
            wait_until = self._t0 + float(Fraction(target) / fps)
            self._clock.sleep(wait_until - self._clock.now())
        else:
            # Consumer ON TIME or BEHIND: deliver the 'now' one; the intermediate ones
            # (next_index .. target-1) are discarded.
            target = n_due

        # 2) Advance the decoder up to `target` with grab() (cheap, without decoding
        #    the skipped ones). If grab() fails, the physical end of the media was reached.
        while self._decoded_index < target:
            if not self._reader.grab():
                self._eof_reached = True
                break
            self._decoded_index += 1

        # 3) If the media ended before reaching `target`, deliver the last available
        #    frame (if any remains undelivered); otherwise, terminate.
        if self._eof_reached and self._decoded_index < target:
            if self._decoded_index > self._delivered_index:
                target = self._decoded_index
            else:
                self._terminated = True
                return Frame(None, self._last_ts, True)

        # 4) Count discards and deliver `target`. The discard = gap between the last
        #    delivered one and the current one.
        self._discarded += target - self._delivered_index - 1
        image = self._reader.retrieve()
        self._delivered_index = target
        self._delivered += 1
        ts = float(Fraction(target) / fps)  # = target * 12/355 (media seconds)
        self._last_ts = ts
        return Frame(image, ts, False)

    # -- metrics ----------------------------------------------------------- #
    @property
    def delivered_frames(self) -> int:
        return self._delivered

    @property
    def discarded_frames(self) -> int:
        return self._discarded

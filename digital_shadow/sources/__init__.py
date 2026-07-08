"""Video sources: contract and implementations.

The pipeline depends only on `VideoSource`/`Frame`. `file_source_from_site` wires the
site config to the file source, so the pipeline obtains a source without knowing where
the parameters come from (nothing hardcoded).
"""

from __future__ import annotations

from typing import Optional

from ..config import SiteConfig
from .base import Clock, Frame, FrameReader, MonotonicClock, VideoSource
from .file_source import FileVideoSource, OpenCVFrameReader

__all__ = [
    "Frame",
    "VideoSource",
    "Clock",
    "MonotonicClock",
    "FrameReader",
    "FileVideoSource",
    "OpenCVFrameReader",
    "file_source_from_site",
]


def file_source_from_site(
    site: SiteConfig, *, clock: Optional[Clock] = None
) -> FileVideoSource:
    """Build (without opening) a `FileVideoSource` from the site config.

    The values (path, fps_ref, expected resolution) all come from config. The caller
    opens with `with` or `.open()`.
    """
    return FileVideoSource(
        video_path=site.video_path,
        fps_ref=site.fps_ref,
        clock=clock,
        expected_resolution=site.resolution,
    )

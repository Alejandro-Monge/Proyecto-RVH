"""Loading of the per-site configuration (`config/sites/<id>/site.yaml`).

Every site-dependent property (video path, fps_ref, resolution, segment window)
lives in config, never embedded in the logic (a design requirement). This module
reads it and delivers it as a typed `SiteConfig`.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Optional

import yaml

# Marker that identifies the project root.
_ROOT_MARKER = "requirements.txt"


def find_project_root(start: Optional[Path] = None) -> Path:
    """Walk up the tree from `start` until the project root is found.

    Locates the root by marker (`requirements.txt`), not by `cwd`, so that the
    config paths resolve the same regardless of where it is run from.
    """
    here = (start or Path(__file__)).resolve()
    for parent in [here, *here.parents]:
        if (parent / _ROOT_MARKER).is_file():
            return parent
    raise RuntimeError(
        f"No se encontró la raíz del proyecto (marcador {_ROOT_MARKER!r}) "
        f"subiendo desde {here}"
    )


@dataclass(frozen=True)
class SiteConfig:
    """Site parameters relevant to the video source and the calibration."""

    site_id: str
    # ABSOLUTE path to the source media (resolved against the project root).
    video_path: Path
    # REAL fps of the media as an exact fraction (e.g. 355/12). Do not round: the
    # frame↔second mapping (and the ground truth) depends on the exact CFR.
    fps_ref: Fraction
    frame_width: int
    frame_height: int
    # Observation window of the segment W = [t0_s, t1_s] in seconds.
    t0_s: float
    t1_s: float
    # Active class filter of the site (COCO names). The detector translates it to
    # ids and filters inference to these classes.
    class_set: tuple[str, ...]
    # Arms of the intersection (site vocabulary).
    arm_set: tuple[str, ...]
    # Site-dependent tracking: frames a track is retained without a match
    # (ByteTrack's lost_track_buffer), fixed by the typical occlusion of the site.
    lost_track_buffer_frames: int
    # Site-dependent re-id gates (scene/occlusion).
    reid_temporal_window_frames: int
    reid_spatial_corridor_px: float

    @property
    def resolution(self) -> tuple[int, int]:
        """(width, height) in pixels."""
        return (self.frame_width, self.frame_height)

    @property
    def expected_total_frames(self) -> int:
        """Total frames of the segment at CFR: (t1 - t0) * fps_ref.

        It is the denominator of the discard rate (protocol Section 5.5/Section 7).
        For Rusia: 180 * 355/12 = 5325, exact because the video is CFR.
        """
        total = Fraction(self.t1_s).limit_denominator() * self.fps_ref
        total -= Fraction(self.t0_s).limit_denominator() * self.fps_ref
        if total.denominator != 1:
            raise ValueError(
                f"El total de frames del tramo no es entero ({total}); "
                "revisa t0_s/t1_s/fps_ref (¿el recorte es CFR?)"
            )
        return int(total)


def load_site_config(
    site_id: str = "rusia", project_root: Optional[Path] = None
) -> SiteConfig:
    """Load and validate `config/sites/<site_id>/site.yaml`."""
    root = project_root or find_project_root()
    site_yaml = root / "config" / "sites" / site_id / "site.yaml"
    if not site_yaml.is_file():
        raise FileNotFoundError(f"No existe la config del sitio: {site_yaml}")

    with open(site_yaml, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    # fps_ref comes as "355/12" (string): Fraction parses it exactly.
    fps_ref = Fraction(str(data["fps_ref"]))
    video_path = (root / data["video_ref"]).resolve()

    return SiteConfig(
        site_id=data["site_id"],
        video_path=video_path,
        fps_ref=fps_ref,
        frame_width=int(data["frame_width"]),
        frame_height=int(data["frame_height"]),
        t0_s=float(data["t0_s"]),
        t1_s=float(data["t1_s"]),
        class_set=tuple(data["class_set"]),
        arm_set=tuple(data["arm_set"]),
        lost_track_buffer_frames=int(data["lost_track_buffer_frames"]),
        reid_temporal_window_frames=int(data["reid_temporal_window_frames"]),
        reid_spatial_corridor_px=float(data["reid_spatial_corridor_px"]),
    )

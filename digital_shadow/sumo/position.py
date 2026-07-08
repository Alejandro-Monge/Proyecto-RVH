"""POSITION piece of the shadow in SUMO (DECOMPOSED position-driven).

Produces, per vehicle, where to place it in SUMO: `(laneID, longitudinal_pos)` for
`traci.vehicle.moveTo(vehID, laneID, pos)`. Design audited with data:

  - **LATERAL = lane (discrete).** The classified lane (image plane, at the stop line) → SUMO lane
    via `lane_map` (already closed); SUMO **centers** the vehicle on that lane. The homography is
    NOT used for the lateral → its lateral error (measured: ~47 % of the energy of the H error, up
    to 86 % in this one) **does not reach SUMO**. It is committed at the **stop-line crossing**
    (causal instant).
  - **LONGITUDINAL = homography + causal filter.** The ground contact is projected to world coords
    and projected onto the **approach edge axis** (from the net, `lane_map.approach_axis`) → `pos`.
    It is SMOOTHED with a causal constant-velocity filter (alpha-beta), SHORT sub-second memory,
    **past only** (zero look-ahead). NOT an average over the reconnection buffer (that would bias
    meters backward).

It satisfies the causal contract (`digital_shadow/causal`): the transform (`project_to_world`,
`longitudinal_pos`) is a PURE lookup (clause i); the filter and the per-vehicle positioner are
**backward-only** state (clause ii). The "when" (when the lane is committed, when it is instantiated)
is decided by the harness, not this piece.

Scope of this step: the **ENTRY** side (approach). The egress/junction are composed in the route
piece. Runtime reads only config (`site.yaml: homography_ref`, `sumo/lane_map.yaml: approach_axis`,
`config/positioning.yaml`); the H and the net are touched only by their derivation/calibration scripts.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from digital_shadow.config.site import find_project_root
from digital_shadow.sumo.lane_map import ApproachAxis, SumoLaneMap, load_lane_map

# ---------------------------------------------------------------------------- #
# Calibration/config loading (runtime reads ONLY config)
# ---------------------------------------------------------------------------- #
def load_homography(site_id: str = "rusia", project_root: Optional[Path] = None) -> np.ndarray:
    """Official image→world homography matrix (3×3), from `site.yaml: homography_ref`."""
    root = project_root or find_project_root()
    with open(root / "config" / "sites" / site_id / "site.yaml", encoding="utf-8") as fh:
        ref = yaml.safe_load(fh)["homography_ref"]
    return np.load(root / ref)


def load_alpha(project_root: Optional[Path] = None) -> float:
    """Alpha gain of the longitudinal filter, from `config/positioning.yaml` (site-independent)."""
    root = project_root or find_project_root()
    with open(root / "config" / "positioning.yaml", encoding="utf-8") as fh:
        return float(yaml.safe_load(fh)["longitudinal_filter"]["alpha"])


# ---------------------------------------------------------------------------- #
# PURE transform (contract clause i): same input → same output, stateless
# ---------------------------------------------------------------------------- #
def project_to_world(H: np.ndarray, px: tuple[float, float]) -> tuple[float, float]:
    """Project an image point (px,py) to world/SUMO coords via the homography (pure, per point)."""
    w = H @ np.array([px[0], px[1], 1.0])
    return float(w[0] / w[2]), float(w[1] / w[2])


def longitudinal_pos(axis: ApproachAxis, world_xy: tuple[float, float]) -> float:
    """Project a world point onto the approach axis → `pos` ∈ [0, length] (pure).

    pos = clip((p − origin)·dir, 0, length). The lateral (perpendicular component) is DISCARDED: it
    is fixed by the lane. For `traci.vehicle.moveTo(laneID, pos)`."""
    o, d = axis.origin, axis.direction
    pos = (world_xy[0] - o[0]) * d[0] + (world_xy[1] - o[1]) * d[1]
    return float(min(max(pos, 0.0), axis.length))


def raw_pos(H: np.ndarray, axis: ApproachAxis, px: tuple[float, float]) -> float:
    """Pure composition: image ground contact → raw longitudinal pos (unfiltered)."""
    return longitudinal_pos(axis, project_to_world(H, px))


def resolve_lane(lane_map: SumoLaneMap, arm: str, label) -> tuple[str, str]:
    """Classified lane label → (SUMO laneID, mode).

    mode: 'exact' (lane resolved), 'grouped' (oeste {2,3}: representative = the rightmost one,
    lane 2 — SENSOR limit), 'fallback' (arm_only/unresolved: lane 1, the rightmost one — declared
    limit e.g. sur, NOT a bug). The fine lateral is limited by the classification, not the pos.
    """
    s = str(label).strip()
    if s.isdigit():
        edge, idx = lane_map.target(arm, "entry", int(s))
        return f"{edge}_{idx}", "exact"
    if "-" in s:                                   # group "2-3": representative = lowest lane (rightmost)
        edge, idx = lane_map.target(arm, "entry", min(int(p) for p in s.split("-")))
        return f"{edge}_{idx}", "grouped"
    edge, idx = lane_map.target(arm, "entry", 1)   # arm_only / unresolved → lane 1
    return f"{edge}_{idx}", "fallback"


# ---------------------------------------------------------------------------- #
# Backward-only state (contract clause ii): filter + per-vehicle positioner
# ---------------------------------------------------------------------------- #
class AlphaBetaFilter:
    """CAUSAL constant-velocity filter (alpha-beta) over a scalar (the longitudinal `pos`).

    Past only: each `observe(t, z)` predicts with the estimated velocity and corrects with the residual.
    `beta = alpha²/(2−alpha)` (critically damped, no overshoot). It does not bias a moving target
    (unlike a simple average). Implements `CausalEstimator`.
    """

    def __init__(self, alpha: float) -> None:
        self.alpha = float(alpha)
        self.beta = self.alpha ** 2 / (2.0 - self.alpha)
        self._x: Optional[float] = None
        self._v = 0.0
        self._t: Optional[float] = None

    def observe(self, t: float, z: float) -> None:
        if self._x is None:                        # initialization with the 1st observation
            self._x, self._t, self._v = float(z), float(t), 0.0
            return
        dt = t - self._t
        if dt <= 0:                                # time does not advance: correct position without velocity
            self._x += self.alpha * (z - self._x)
            self._t = t
            return
        xp = self._x + self._v * dt                # prediction
        r = z - xp                                 # residual
        self._x = xp + self.alpha * r
        self._v += (self.beta / dt) * r
        self._t = t

    def value(self) -> float:
        """current filtered pos (valid after ≥1 observation)."""
        if self._x is None:
            raise ValueError("AlphaBetaFilter.value() antes de cualquier observe()")
        return self._x

    @property
    def velocity(self) -> float:
        return self._v


@dataclass
class Placement:
    """Placement of a vehicle in SUMO on a frame. `lane_id` is None until the lane is committed (at
    the stop-line crossing); `pos` (filtered longitudinal) is available from birth."""

    lane_id: Optional[str]
    pos: float
    lane_mode: Optional[str] = None  # exact|grouped|fallback (when lane_id is already set)


class VehiclePositioner:
    """Per-vehicle state (backward-only). The LONGITUDINAL is filtered from birth over the approach
    axis (the arm is known early); the LATERAL (lane→laneID) is committed at the stop-line crossing.
    It reflects the decomposition: independent axes, both causal.

    The harness calls `observe(t, ground_contact_px)` every frame and `commit_lane(label)` at the
    crossing; `placement()` gives `(laneID, pos)` for `moveTo`. The "when" belongs to the harness,
    not here.
    """

    def __init__(self, arm: str, lane_map: SumoLaneMap, H: np.ndarray, alpha: float) -> None:
        self.arm = arm
        self._lane_map = lane_map
        self._H = H
        self._axis = lane_map.approach_axis[arm]
        self._filter = AlphaBetaFilter(alpha)
        self._lane_id: Optional[str] = None
        self._lane_mode: Optional[str] = None

    def observe(self, t: float, ground_contact_px: tuple[float, float]) -> None:
        """Integrate the frame's ground contact into the longitudinal filter (past only)."""
        self._filter.observe(t, raw_pos(self._H, self._axis, ground_contact_px))

    def commit_lane(self, label) -> None:
        """Commit the lane (at the stop-line crossing) → SUMO laneID."""
        self._lane_id, self._lane_mode = resolve_lane(self._lane_map, self.arm, label)

    def placement(self) -> Placement:
        """Current placement: (laneID — None until the crossing, filtered longitudinal pos)."""
        return Placement(self._lane_id, self._filter.value(), self._lane_mode)

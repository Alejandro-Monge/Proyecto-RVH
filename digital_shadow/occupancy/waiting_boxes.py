"""Waiting boxes: causal PRESENTATION layer of the approach occupancy.

Draws (via TraCI polygons, not routed vehicles) a rectangular silhouette per vehicle
WAITING in the arm before the stop line: presence + detector class, ANCHORED TO THE
LANE with the doctrine of position.py — LATERAL fixed to the center of the voted lane
(the homography is not used for the lateral; its lateral error, ~47 % of the energy,
does not reach the box), LONGITUDINAL = homographic projection onto the lane axis
(longitudinal_pos) + causal alpha-beta filter from config. In the background with no
lane zone: raw homographic position (DECLARED fallback, not dressed up). Orientation
by the GEOMETRIC direction of the lane (nose to the stop line), dimension and color
of the REAL SUMO vTypes (exact identity with the causal vehicle that is born when
crossing the stop line — seam of the next phase).

CAUSALITY (same contract as the operating point of the measurement layer,
approach_zones.yaml: measurement): the box appears after θ seconds of ACCUMULATED
dwell in the zone (a condition over the past) and survives presence gaps ≤ g
(bounded persistence when the point is lost, like the R2 removal). The class is the
majority vote from the track's PAST. Nothing reads the future. Extinction: on reaching
the stop-line crossing of its causal chain (the routed vehicle is born), on exhausting
g without presence, or when the track ends.

Does NOT touch the measurement: the readouts read vehicle.* and the matcher reads
chains; the polygons do not participate in car-following nor in insertion checks (SUMO
VehicleInsertion doc). Overlap of boxes in a dense queue: it is drawn AS IS — it is the
real perception uncertainty of the far field, not dressed up.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field

import numpy as np

from digital_shadow.sumo.lane_map import ApproachAxis
from digital_shadow.sumo.position import longitudinal_pos

# Fill alpha (0-255): translucent fill in the exact vType color;
# the solid causal vehicle stands out when it is born.
FILL_ALPHA = 120


def lane_axis_from_shape(shape_points) -> ApproachAxis:
    """LANE axis from its net polyline, to anchor the box (doctrine of position.py
    applied to the visual layer). Origin→endpoint reduction VERIFIED exact: the 12 entry
    lanes of the net are 2-point straight lines; if a future site brings curved lanes,
    this reduction no longer holds and one must project onto the full polyline."""
    p0 = np.asarray(shape_points[0], float)
    p1 = np.asarray(shape_points[-1], float)
    d = p1 - p0
    length = float(np.hypot(*d))
    d = d / length
    return ApproachAxis(origin=(float(p0[0]), float(p0[1])),
                        direction=(float(d[0]), float(d[1])), length=length)


def anchor_to_lane(axis: ApproachAxis, world_xy) -> tuple[float, float]:
    """LONGITUDINAL = homographic projection onto the lane axis (longitudinal_pos,
    pure lookup); LATERAL = discarded → center of the lane. The lateral error of the
    homography (~47 % of its energy, up to 86 % in this one) does not reach the box."""
    pos = longitudinal_pos(axis, world_xy)
    return (axis.origin[0] + pos * axis.direction[0],
            axis.origin[1] + pos * axis.direction[1])


def lane_angle_towards_stop(shape_points) -> float:
    """Angle (rad, mathematical convention x→east) of the LAST segment of the entry lane
    polyline = direction of travel towards the stop line (lane.getShape goes from the
    start of the edge towards the junction)."""
    (x0, y0), (x1, y1) = shape_points[-2], shape_points[-1]
    return float(np.arctan2(y1 - y0, x1 - x0))


def box_corners(nose_xy, angle: float, length: float, width: float) -> np.ndarray:
    """Corners (4,2) of the rectangle with the NOSE at nose_xy and the body backwards —
    same convention as SUMO (the vehicle position is its front), so that the box and the
    causal vehicle coincide on the same homographic point."""
    d = np.array([np.cos(angle), np.sin(angle)])          # towards the stop line
    n = np.array([-d[1], d[0]])                            # perpendicular
    nose = np.asarray(nose_xy, float)
    rear = nose - length * d
    h = 0.5 * width * n
    return np.array([nose + h, nose - h, rear - h, rear + h])


@dataclass
class _TrackState:
    presence_s: float = 0.0            # accumulated dwell in the zone (past)
    last_seen: float = -1.0
    last_xy: tuple = (0.0, 0.0)
    cls_votes: Counter = field(default_factory=Counter)
    lane_votes: Counter = field(default_factory=Counter)   # lane vote from the PAST


class WaitingBoxTracker:
    """Causal state of the boxes. update(t, present) per frame, with
    present = iterable of (track_id, x, y, class_id, carril) ALREADY filtered to arm zone
    (carril = int of the lane zone, or None if it falls in the arm outside lanes).
    Returns {track_id: (x, y, majority_class_id, majority_lane_or_None)} of the visible
    boxes. The past lane vote stabilizes the zone-boundary flips (avoids lateral jumps of
    the box between lane centers) and is causal."""

    def __init__(self, theta_s: float, gap_s: float, cross_t_by_track: dict[int, float],
                 frame_dt: float):
        self.theta = float(theta_s)
        self.gap = float(gap_s)
        self.cross_t = dict(cross_t_by_track)
        self.dt = float(frame_dt)
        self._st: dict[int, _TrackState] = {}

    def has_lane_votes(self, tid) -> bool:
        """Does the track already have a lane assignment (past vote)? For the c2-sur rule:
        the c1-empty condition is evaluated only WHEN ASSIGNING; once the vote is injected,
        the existing majority keeps the lane while the box lives (the reset by gap>g reopens
        the evaluation in the new life). Causal: only past state."""
        s = self._st.get(int(tid))
        return bool(s and s.lane_votes)

    def update(self, t: float, present) -> dict[int, tuple]:
        for tid, x, y, cls, carril in present:
            tid = int(tid)
            ct = self.cross_t.get(tid)
            if ct is not None and t >= ct:
                continue                      # already crossed the stop line: the causal one is born, not the box
            s = self._st.setdefault(tid, _TrackState())
            if s.last_seen >= 0 and (t - s.last_seen) > self.gap:
                # gap larger than g: reset the dwell (re-appeared track)
                s.presence_s = 0.0
                s.cls_votes = Counter()
                s.lane_votes = Counter()
            s.presence_s += self.dt
            s.last_seen = t
            s.last_xy = (float(x), float(y))
            s.cls_votes[int(cls)] += 1
            if carril is not None:
                s.lane_votes[int(carril)] += 1
        out = {}
        for tid in list(self._st):
            s = self._st[tid]
            ct = self.cross_t.get(tid)
            if (t - s.last_seen) > self.gap or (ct is not None and t >= ct):
                del self._st[tid]             # extinction: gap exhausted or stop-line crossing
                continue
            if s.presence_s >= self.theta:
                lane = s.lane_votes.most_common(1)[0][0] if s.lane_votes else None
                out[tid] = (*s.last_xy, s.cls_votes.most_common(1)[0][0], lane)
        return out

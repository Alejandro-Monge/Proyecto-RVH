"""Ground-contact correction by the vehicle's 3D BOX (length-aware) — ONLY for the EXIT lane.

PROBLEM (measured): the detector's 2D box wraps the vehicle's 3D VOLUME; its center-x (`cx`) is shifted
from the point where the vehicle TREADS on the lane. The bias is DOMINATED by the LENGTH (a long box
projected obliquely), not the height: at the east egress, a 12 m bus biases `s` ~+20.9 px (n=8, ~80 % of
the lane width) → the buses are classified in the neighbor lane. Decomposition (east egress lane 1):
car +14.1 px (length +16.7), bus +37.0 px (length +43.7); the height is smaller and of opposite sign.

CORRECTION: the REAL ground contact is estimated by projecting the 3D box (dimensions per vType class:
bus 12×3.2, truck 8×3.5, car 4.5×1.5 m) with the camera recovered from the site HOMOGRAPHY, oriented by
the EGRESS LANE. It is a PER-SITE CALIBRATION PARAMETER (the camera comes from the Rusia homography),
PHYSICAL, not fitted to the observed buses (n=8 only verified that the model +22.9 ≈ the measured bias +20.9).

SCOPE: ONLY the EXIT lane. The bias is ASYMMETRIC — exit +20.9 px real (n=8); entry ~0 (n=1, does not
allow an assertion). Applying it to the entry would introduce a bias (−11.8) that did not exist. That is
why the ENTRY classification uses the raw ground contact, untouched.

STATUS: **NOT INTEGRATED in production**. The physics is validated (the length bias +20.9 px on east exit
buses, model +22.9), but the integration does NOT transfer:
  - Correcting without re-calibrating bands breaks the cars (they carry the +16.7 bias baked into the
    bands; west exit 83%→67%).
  - Re-calibrating bands {1:24.6,2:59.4} over the corrected contact + restricting to east exit gives 96%
    (LOO) ONLY if measured over the RECONNECTED tracks (batch); run_core uses the RAW chains of the CAUSAL
    orchestrator — a different ground contact at the crossing — and there it does NOT rise (east stays
    ~82-87 %, WITH ≈ WITHOUT). The 96% was a DATA-SOURCE artifact (reconnected ≠ raw).
For a valid integration: calibrate AND verify over the SAME source that run_core uses (the causal
orchestrator chains), not the reconnected-batch tracks. This module remains as a verified physical
basis; the transfer to production is the open problem.

I/O CONTRACT:
  - `BoxGroundCorrector(H, net_path, lane_map, center)`: recovers the camera from the homography and reads
    the egress orientations from the net (no offset; netOffset 0,0).
  - `.corrected_exit_s(chdf, stop_line, center, vehicle_class, arm) -> float|None`: the transverse `s` of
    the egress crossing (lane_cross_coord) MINUS the modeled box bias. None if the track does not cross.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from digital_shadow.sumo.position import project_to_world

# Physical dimensions (width, length, height) in meters, per vType class. The LENGTH is the lever.
VTYPE_DIMS = {"car": (1.8, 4.5, 1.5), "bus": (2.5, 12.0, 3.2), "truck": (2.6, 8.0, 3.5)}


def recover_camera(H: np.ndarray, u0: float, v0: float) -> np.ndarray:
    """World→image projection matrix (3×4) recovered from the image→world homography, assuming the
    principal point at (u0,v0) and `f` from the orthogonality of the homography (h1⊥h2, ‖h1‖=‖h2‖).
    First-principles."""
    Hwi = np.linalg.inv(H)
    h1, h2 = Hwi[:, 0], Hwi[:, 1]
    C = np.array([[1, 0, -u0], [0, 1, -v0], [-u0, -v0, u0 ** 2 + v0 ** 2]], float)
    f = float(np.sqrt(-(h1 @ C @ h2) / (h1[2] * h2[2])))
    K = np.array([[f, 0, u0], [0, f, v0], [0, 0, 1.0]])
    Ki = np.linalg.inv(K)
    r1 = Ki @ h1
    lam = 1.0 / np.linalg.norm(r1)
    r1 = r1 * lam
    r2 = lam * (Ki @ h2)
    t = lam * (Ki @ Hwi[:, 2])
    return K @ np.column_stack([r1, r2, np.cross(r1, r2), t])


def _egress_dirs(net_path, lane_map) -> dict:
    """Orientation (world) of the EGRESS lane of each arm, from the net (first segment, leaving the
    junction). The Rusia net has netOffset 0,0 → XML coords = homography coords."""
    r = ET.parse(net_path).getroot()
    shapes = {}
    for e in r.findall("edge"):
        for l in e.findall("lane"):
            shapes[l.get("id")] = np.array([[float(c) for c in q.split(",")] for q in l.get("shape").split()])
    out = {}
    for arm in lane_map.arms:
        lane_id = lane_map.arms[arm].exit_edge + "_0"
        s = shapes.get(lane_id)
        if s is not None and len(s) >= 2:
            d = s[1] - s[0]
            out[arm] = d / np.linalg.norm(d)
    return out


class BoxGroundCorrector:
    def __init__(self, H: np.ndarray, net_path, lane_map, center: np.ndarray):
        self.H = H
        self.P = recover_camera(H, float(center[0]), float(center[1]))
        self.egress = _egress_dirs(net_path, lane_map)

    def _proj(self, W, Z):
        q = self.P @ np.array([W[0], W[1], Z, 1.0])
        return q[:2] / q[2]

    def _exit_s_bias(self, cx, y2, vehicle_class, arm, stop_line):
        """Bias in `s` (transverse to the stop line) = s(center of the 3D box) − s(real contact), at the egress."""
        d = self.egress.get(arm)
        if d is None:
            return 0.0
        wd, L, Hh = VTYPE_DIMS.get(vehicle_class, VTYPE_DIMS["car"])
        W = np.array(project_to_world(self.H, (float(cx), float(y2))))
        n = np.array([-d[1], d[0]])
        p1 = stop_line[0]
        u = stop_line[1] - stop_line[0]
        u = u / np.linalg.norm(u)
        pts = np.array([self._proj(W + sx * (wd / 2) * n + sy * (L / 2) * d, z)
                        for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, Hh)])
        box_c = np.array([(pts[:, 0].min() + pts[:, 0].max()) / 2, pts[:, 1].max()])
        true = self._proj(W, 0.0)
        return float((box_c - p1) @ u) - float((true - p1) @ u)

    def corrected_exit_s(self, chdf, stop_line, center, vehicle_class, arm, dte):
        """`s` of the EGRESS crossing (lane_cross_coord) corrected by the box bias. None if it does not cross."""
        s = dte.lane_cross_coord(chdf, stop_line, center, kind="exit")
        if s is None:
            return None
        p1, nh = dte._inward_normal(stop_line, center)
        pts = chdf[["gx", "gy"]].to_numpy()
        dn = (pts - p1) @ nh
        idx = [i for i in range(len(dn) - 1) if dn[i] > 0 >= dn[i + 1]]
        if not idx:
            return s
        i = idx[-1]
        lo, hi = max(0, i - 2), min(len(pts), i + 4)
        bias = float(np.median([self._exit_s_bias(a, b, vehicle_class, arm, stop_line) for a, b in pts[lo:hi]]))
        return s - bias

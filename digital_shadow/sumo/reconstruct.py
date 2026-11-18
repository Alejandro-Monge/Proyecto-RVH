"""Reconstruction of the trajectory in SUMO by LANE GEOMETRY — STEP 1 of the background correction.

SUMO is where the shadow LIVES: the reconstructed trajectory must be faithful, not the raw H
projection (which is ~1 lane off per zone). SHAPE / ADVANCE decomposition:

  - SHAPE = a CONTINUOUS PATH of the net (clean, no drift): CLASSIFIED entry lane → internal lane(s)
    of the OBSERVED turn (full walks, including the `via` of left turns) → egress lane. Concatenated
    end-to-start: the approach→junction→egress transitions are smooth by construction (a single path;
    no boundary jumps).
  - ADVANCE = the observed trail PROJECTED onto the path (arc length), monotonic and filtered
    (causal). Robust to lateral drift (⊥ to the advance, verified 0.00 m). RESPECTS STOPS: if the
    trail stops, the arc length stops → the vehicle stays parked on the curve.

GROUPED lanes (oeste 2-3, the camera does not separate them): the path entry = LATERAL MEAN of the
group's lanes (honest), not a forced representative. It does NOT include buffer reconciliation in the
gaps (step 2): a gap with no reappearance is covered by the filter extrapolation / render interpolation.
"""
from __future__ import annotations

import numpy as np

from digital_shadow.sumo.stage import point_at_arclen, polyline_arclen


def _resample(shape: np.ndarray, k: int = 24) -> np.ndarray:
    """Resample a polyline to `k` points equispaced in arc length."""
    arc = polyline_arclen(shape)
    grid = np.linspace(0.0, arc[-1], k)
    return np.array([point_at_arclen(shape, s) for s in grid])


def _densify(shape: np.ndarray, step: float = 1.0) -> np.ndarray:
    """Resample at step ~`step` m (short segments → the windowed projection stays bounded)."""
    L = float(polyline_arclen(shape)[-1])
    grid = np.linspace(0.0, L, max(int(L / step) + 1, 2))
    return np.array([point_at_arclen(shape, s) for s in grid])


def _mean_entry_shape(traci, lane_map, entry_arm: str, carriles) -> np.ndarray:
    """Lateral entry shape: the classified lane, or the MEAN of the group's lanes (grouped)."""
    edge = lane_map.arms[entry_arm].entry_edge
    shapes = [np.asarray(traci.lane.getShape(f"{edge}_{lane_map.arms[entry_arm].entry_lanes[c]}"), float)
              for c in carriles if c in lane_map.arms[entry_arm].entry_lanes]
    if not shapes:
        shapes = [np.asarray(traci.lane.getShape(f"{edge}_0"), float)]
    if len(shapes) == 1:
        return shapes[0]
    return np.mean([_resample(s) for s in shapes], axis=0)   # lateral mean of the group


def _internal_chain(traci, internal_id: str):
    """Walk the internal lanes from `internal_id` to the egress lane (non-internal). Returns
    (list of internal shapes, egress laneID). Covers the `via` of left turns."""
    shapes, cur, seen = [], internal_id, set()
    egress = None
    while cur and cur.startswith(":") and cur not in seen:
        seen.add(cur)
        shapes.append(np.asarray(traci.lane.getShape(cur), float))
        links = traci.lane.getLinks(cur)
        cur = links[0][0] if links else None
    egress = cur                                            # first non-internal lane = egress
    return shapes, egress


def _movement_connections(traci, lane_map, entry_arm: str, exit_arm: str):
    """REAL net connections for the movement (entry_arm→exit_arm), read from traci:
    list of {from_carril, from_idx, to_carril, to_idx, via}. It is the basis of the
    data-dependent egress: choose the connection whose `to_carril` = observed exit lane."""
    ee = lane_map.arms[entry_arm].entry_edge
    xe = lane_map.arms[exit_arm].exit_edge
    ex_idx2c = {i: c for c, i in lane_map.arms[exit_arm].exit_lanes.items()}
    out = []
    for c, i in lane_map.arms[entry_arm].entry_lanes.items():
        try:
            links = traci.lane.getLinks(f"{ee}_{i}")
        except Exception:
            continue
        for lk in links:
            appr, via = lk[0], lk[4]                       # approximate egress lane, internal lane (via)
            if appr.rsplit("_", 1)[0] == xe:
                ti = int(appr.rsplit("_", 1)[1])
                out.append({"from_carril": c, "from_idx": i, "to_carril": ex_idx2c.get(ti),
                            "to_idx": ti, "via": via})
    return out


def choose_connection(traci, lane_map, entry_arm, exit_arm, carriles, exit_carril=None):
    """ONE net connection choice for the movement, shared by the reconstruction (recon) and by the
    render routing (departLane): this way the entry, the internal curve, and the egress ALWAYS come
    from the SAME lane, and the SIM route matches the recon (closes the recon-vs-SIM mismatch).

      1) Connection whose `from_carril` = observed ENTRY lane; among those, the one with `to_carril` =
         observed EXIT lane (data-dependent egress). Covers LEGAL and ROUTABLE.
      2) If NO connection departs from the observed lane (ORPHAN): the observed tuple is IMPOSSIBLE in
         the net, which signals an UNRELIABLE OBSERVATION. It falls back to the COMPLETE CANONICAL
         connection of the movement (`via` = `junction_internal`): entry + internal + egress of the
         SAME canonical lane, without a jump. An HONEST maneuver without knowing the future or the GT;
         here `exit_carril` is NOT used (it comes from the same unreliable observation). That it matches
         the GT on this segment is a RESULT that is reported, NOT the design criterion.

    Returns {from_carril, from_idx, to_carril, to_idx, via}, or None only if the movement does not exist
    in the net nor as canonical (degenerate)."""
    conns = _movement_connections(traci, lane_map, entry_arm, exit_arm)
    cand = [c for c in conns if c["from_carril"] in carriles]
    if cand:
        pref = [c for c in cand if c["to_carril"] == exit_carril]
        return (pref or cand)[0]
    canon_via = lane_map.junction_internal.get((entry_arm, exit_arm))
    return next((c for c in conns if c["via"] == canon_via), None)


def build_path(traci, lane_map, entry_arm: str, exit_arm: str, carriles, exit_carril=None,
               conn=None) -> np.ndarray:
    """Continuous net path: entry + internal lanes of the turn + egress, ALL from the SAME lane (the
    unified connection from `choose_connection`). Concatenated end-to-start (smooth transitions by
    construction).

    `conn` (optional): the connection already chosen by the render, so that recon and routing
    (departLane) use EXACTLY the same one → the recon-vs-SIM mismatch closes to ~0. If not given, it is
    chosen here. `exit_carril` influences only through `choose_connection` (data-dependent egress in
    LEGAL/ROUTABLE; ignored in the canonical fallback of the orphans)."""
    if conn is None:
        conn = choose_connection(traci, lane_map, entry_arm, exit_arm, carriles, exit_carril)
    if conn is None:                                   # degenerate: movement nonexistent in the net
        return _mean_entry_shape(traci, lane_map, entry_arm, carriles)   # last resort: only the approach
    ee = lane_map.arms[entry_arm].entry_edge
    entry = np.asarray(traci.lane.getShape(f"{ee}_{conn['from_idx']}"), float)
    internal_shapes, egress_id = _internal_chain(traci, conn["via"])
    parts = [entry, *internal_shapes]
    if egress_id:
        parts.append(np.asarray(traci.lane.getShape(egress_id), float))
    # concatenate, avoiding duplicate vertices at the joins
    path = [parts[0]]
    for p in parts[1:]:
        if np.hypot(*(p[0] - path[-1][-1])) < 0.05:
            p = p[1:]
        path.append(p)
    return _densify(np.vstack(path), step=1.0)   # dense: the windowed projection stays bounded


def _nearest_arclen(path: np.ndarray, arc: np.ndarray, w: np.ndarray, lo: float, hi: float) -> float:
    """Arc length of the path point nearest to w, restricted to [lo, hi] (monotonic progression)."""
    best_d, best_s = 1e18, lo
    for i in range(len(path) - 1):
        if arc[i + 1] < lo or arc[i] > hi:
            continue
        a, b = path[i], path[i + 1]
        ab = b - a
        denom = float(ab @ ab) or 1e-9
        t = min(max(float((w - a) @ ab) / denom, 0.0), 1.0)
        q = a + t * ab
        d = float(np.hypot(*(w - q)))
        if d < best_d:
            best_d, best_s = d, float(arc[i] + t * np.hypot(*ab))
    return min(max(best_s, lo), hi)              # clamp to the window (short segments, safe)


def _tangent_at(path: np.ndarray, arc: np.ndarray, s: float) -> np.ndarray:
    """Unit direction of the path at arc length `s`."""
    i = int(np.searchsorted(arc, s) - 1)
    i = max(0, min(i, len(path) - 2))
    d = path[i + 1] - path[i]
    return d / (np.hypot(*d) + 1e-9)


def reconstruct_world(raw_xy: np.ndarray, ts: np.ndarray, path: np.ndarray,
                      reconcile: bool = True, dt_frame: float = 12.0 / 355):
    """Trajectory reconstructed onto the net path. Returns (recon_t, recon_xy).

    ADVANCE (arc length `s`): on CONTIGUOUS frames it accumulates the FORWARD COMPONENT of the observed
    displacement (the lateral jitter cancels out; there is no curve bias; it respects stops). On the
    REAPPEARANCE after a GAP, it anchors `s` to the REAL reappearance POSITION (projection) — it does
    not extrapolate blindly.

    `reconcile=True` (STEP 2): fills each gap by interpolating `s` over the CURVE between the two real
    tips (pre-gap and reappearance) → the vehicle FOLLOWS the curve in the gap, it does not cut it in a
    straight line. `reconcile=False`: only the observed frames (the render interpolates the position →
    cuts the curve).

    Causal cost of the reconciled filling: it uses the reappearance → it adds delay = however long it
    takes to reappear (≤ buffer; the intra-chain gaps are already ≤ buffer thanks to the reconnector).
    The buffer made explicit.
    """
    n = len(raw_xy)
    if path is None or len(path) < 2:
        return np.asarray(ts, float), np.asarray(raw_xy, float)
    arc = polyline_arclen(path)
    L = float(arc[-1])
    obs = np.asarray(raw_xy, float).copy()
    if n >= 3:                                # pre-smooths the 1-frame jitter at the source
        obs[:, 0] = [np.median(raw_xy[max(0, i - 1):i + 2, 0]) for i in range(n)]
        obs[:, 1] = [np.median(raw_xy[max(0, i - 1):i + 2, 1]) for i in range(n)]
    s = np.zeros(n)
    is_gap_end = np.zeros(n, bool)
    s[0] = _nearest_arclen(path, arc, obs[0], 0.0, L)
    for i in range(1, n):
        if float(ts[i] - ts[i - 1]) > 1.5 * dt_frame:          # GAP → anchor to the real reappearance
            s[i] = _nearest_arclen(path, arc, obs[i], s[i - 1], s[i - 1] + 25.0)
            is_gap_end[i] = True
        else:                                                  # contiguous → monotonic projection on path
            s[i] = _nearest_arclen(path, arc, obs[i], s[i - 1], L)
    h = 2
    s = np.array([np.median(s[max(0, i - h):i + h + 1]) for i in range(n)])
    if not reconcile:
        return np.asarray(ts, float), np.array([point_at_arclen(path, float(si)) for si in s])
    # RECONCILIATION: densify the gaps with points ON THE CURVE (s interpolated between the tips)
    dt_t, dt_xy = [float(ts[0])], [point_at_arclen(path, float(s[0]))]
    for i in range(1, n):
        if is_gap_end[i]:
            steps = max(int(round((ts[i] - ts[i - 1]) / dt_frame)), 1)
            for k in range(1, steps):
                f = k / steps
                sk = s[i - 1] + f * (s[i] - s[i - 1])          # linear in ARC LENGTH (follows the curve)
                dt_t.append(float(ts[i - 1] + f * (ts[i] - ts[i - 1])))
                dt_xy.append(point_at_arclen(path, float(sk)))
        dt_t.append(float(ts[i]))
        dt_xy.append(point_at_arclen(path, float(s[i])))
    return np.asarray(dt_t), np.asarray(dt_xy)

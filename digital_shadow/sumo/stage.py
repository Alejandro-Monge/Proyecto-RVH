"""SUMO stage of the shadow — SUMO as a position-driven RENDERER (not a simulator).

`SumoStage` wraps TraCI so that SUMO is a canvas where the shadow DICTATES the position of each
vehicle, not where SUMO drives. Hard requirements (verified):

  - **SUMO control OFF** per vehicle: `setLaneChangeMode(0)` + `setSpeedMode(0)` → SUMO does not
    change lane nor apply car-following; it respects the dictated position (verified: `moveTo`
    faithful to 0.026 m with control off; with control on SUMO deviates ~1 lane).
  - **SUMO slave to the video clock:** `sync(t)` = `simulationStep(float(t))` re-points to the
    ABSOLUTE video time (verified: 0 ms of drift over 5325 frames; dissolves 30 Hz vs 29.583).
  - **Gap filling by the shape of the INTERNAL lane** (not delegated to SUMO, which with control off
    does not advance): the pos is interpolated over the polyline of the internal lane (from
    `lane_map.junction_internal` → `traci.lane.getShape`) at the last observed velocity.

Life cycle: `add` (instantiate) · `place_lane`/`place_xy` (move) · `remove` (retire). GUI mode
(`gui=True`) for the visualization; headless for the measurement. Imports TraCI lazily.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np

# --------------------------------------------------------------------------- #
# Filling interpolation (PURE — no SUMO, testable in isolation)
# --------------------------------------------------------------------------- #
def polyline_arclen(shape) -> np.ndarray:
    """Accumulated arc at each vertex of the polyline (shape = list of (x,y))."""
    p = np.asarray(shape, float)
    seg = np.hypot(*np.diff(p, axis=0).T)
    return np.concatenate([[0.0], np.cumsum(seg)])


def point_at_arclen(shape, arc: float) -> tuple[float, float]:
    """Point (x,y) at a distance `arc` along the polyline (clip to [0, length])."""
    p = np.asarray(shape, float)
    L = polyline_arclen(shape)
    arc = float(min(max(arc, 0.0), L[-1]))
    i = int(np.searchsorted(L, arc) - 1)
    i = max(0, min(i, len(p) - 2))
    f = 0.0 if L[i + 1] == L[i] else (arc - L[i]) / (L[i + 1] - L[i])
    q = p[i] + f * (p[i + 1] - p[i])
    return float(q[0]), float(q[1])


class GapFiller:
    """Advances a vehicle along the shape of the internal lane at constant velocity (the last observed
    one), to fill a junction gap WITHOUT delegating to SUMO. Backward-only state: it only integrates
    the time elapsed since the start of the gap."""

    def __init__(self, shape, speed: float, t0: float, arc0: float = 0.0) -> None:
        self.shape = np.asarray(shape, float)
        self.speed = float(speed)
        self.t0 = float(t0)
        self.arc0 = float(arc0)
        self._len = float(polyline_arclen(shape)[-1])

    def at(self, t: float) -> tuple[float, float]:
        """(x,y) at instant t (≥ t0). The arc advances speed·(t−t0) from arc0."""
        return point_at_arclen(self.shape, self.arc0 + self.speed * (t - self.t0))

    @property
    def length(self) -> float:
        return self._len


# --------------------------------------------------------------------------- #
# SUMO stage
# --------------------------------------------------------------------------- #
class SumoStage:
    """SUMO as a position-driven renderer (control off, slave clock)."""

    def __init__(self, net_path, *, gui: bool = False, step_length: float = 0.001,
                 lane_map=None, vtype: str = "DEFAULT_VEHTYPE") -> None:
        self._net = str(net_path)
        self._bin = "sumo-gui" if gui else "sumo"
        self._step = step_length
        self._lm = lane_map
        self._vtype = vtype
        self._traci = None
        self._routes: set[str] = set()

    def start(self) -> "SumoStage":
        os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
        import traci
        self._traci = traci
        cmd = [self._bin, "-n", self._net, "--step-length", str(self._step),
               "--no-step-log", "true", "--no-warnings", "true",
               # POSITION-DRIVEN: we dictate the position via moveToXY. SUMO must NOT REMOVE
               # vehicles on its own (collision on overlap, teleport due to jam, route errors);
               # if it does, the vehicle disappears in the middle of the shadow. Those removals
               # are disabled.
               "--collision.action", "none", "--time-to-teleport", "-1",
               "--ignore-route-errors", "true"]
        if self._bin == "sumo-gui":
            cmd += ["--start", "true", "--quit-on-end", "true"]
        traci.start(cmd)
        return self

    def add(self, vid: str, edge: str, lane_index: int, pos: float = 0.0, speed: float = 0.0,
            vtype: Optional[str] = None) -> None:
        """Instantiate a vehicle at (edge, lane_index, pos) with CONTROL DISABLED. `vtype` allows
        distinguishing car/bus/truck in the GUI (visual fidelity; does not affect the measurement)."""
        t = self._traci
        rid = f"r_{edge}"
        if rid not in self._routes:
            t.route.add(rid, [edge])
            self._routes.add(rid)
        try:
            t.vehicle.add(vid, rid, typeID=vtype or self._vtype, departLane=str(lane_index),
                          departPos=str(float(pos)), departSpeed=str(float(speed)))
        except Exception:
            t.vehicle.add(vid, rid, typeID=vtype or self._vtype, departLane="free",
                          departPos="0", departSpeed="0")
        t.simulationStep()                       # inserts the vehicle
        t.vehicle.setLaneChangeMode(vid, 0)      # NO autonomous lane change
        t.vehicle.setSpeedMode(vid, 0)           # NO car-following/safety

    def set_target(self, vid: str, exit_edge: str) -> None:
        """changeTarget to the exit edge (logical route / O-D; the position is dictated by us)."""
        self._traci.vehicle.changeTarget(vid, exit_edge)

    def place_lane(self, vid: str, lane_id: str, pos: float) -> None:
        """Place at (lane_id, pos) — approach/egress (position piece)."""
        self._traci.vehicle.moveTo(vid, lane_id, pos)

    def place_xy(self, vid: str, x: float, y: float, *, keep_route: int = 2,
                 edge: str = "", lane: int = -1) -> None:
        """Place at exact (x,y) — junction/filling (keep_route=2 = exact xy without snap; the smoke
        test confirms the correct bit)."""
        self._traci.vehicle.moveToXY(vid, edge, lane, x, y, keepRoute=keep_route)

    def internal_shape(self, entry_arm: str, exit_arm: str):
        """Shape (polyline) of the internal lane of the movement, requested from SUMO (not the net file)."""
        lane = self._lm.internal_lane(entry_arm, exit_arm) if self._lm else None
        if lane is None:
            return None
        return self._traci.lane.getShape(lane)

    def sync(self, t: float) -> float:
        """Re-points the SUMO clock to the ABSOLUTE video time t (no drift). Returns getTime."""
        self._traci.simulationStep(float(t))
        return self._traci.simulation.getTime()

    def remove(self, vid: str) -> None:
        self._traci.vehicle.remove(vid)

    def position(self, vid: str) -> tuple[float, float]:
        return tuple(self._traci.vehicle.getPosition(vid))

    def lane_of(self, vid: str) -> str:
        return self._traci.vehicle.getLaneID(vid)

    @property
    def vehicles(self) -> list:
        return list(self._traci.vehicle.getIDList())

    def close(self) -> None:
        if self._traci is not None:
            self._traci.close()
            self._traci = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()
